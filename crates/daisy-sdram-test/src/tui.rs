//! The live memory-test TUI, rendered over USB-CDC with `ratatui-serial`.
//!
//! Styled after memtest86 / Adrian Black-era diagnostics: a title bar, the
//! target geometry, a live progress gauge for the current March element, a
//! per-group test list with pass/fail marks, running statistics (pass count,
//! elapsed, throughput), and a scrolling error log that localises each failure
//! to an address and the failing data bits (`Dn`).
//!
//! HARDWARE-ONLY (needs real USB; Renode has no OTG model). Open the Daisy's CDC
//! port (`picocom -b 115200 /dev/cu.usbmodem…`) and it redraws live. The
//! `SerialBackend` diffs cells between frames, so a ticking progress bar only
//! sends the few bytes that changed. Drawn on an opaque top-left block at a
//! self-sufficient default size (picocom / macOS Terminal don't reliably answer
//! a size query); clients that DO answer are resized to their real dimensions.

extern crate alloc;

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use ratatui_core::backend::Backend as _;
use ratatui_core::buffer::Buffer;
use ratatui_core::layout::Rect;
use ratatui_core::style::{Color, Modifier, Style};
use ratatui_core::terminal::Terminal;
use ratatui_core::widgets::Widget;
use ratatui_serial::{CprParser, SerialBackend};

use sdram_march::{fault_bits, GROUPS, NGROUPS};

/// How many recent failures the error log keeps.
pub const ERRLOG_CAP: usize = 8;

/// A ring of the most recent failures.
pub struct ErrLog {
    buf: [(u32, u32, u32); ERRLOG_CAP], // (addr, expected, got)
    len: usize,
    head: usize,
    pub total: u32,
}

impl ErrLog {
    pub const fn new() -> Self {
        Self {
            buf: [(0, 0, 0); ERRLOG_CAP],
            len: 0,
            head: 0,
            total: 0,
        }
    }
    pub fn push(&mut self, addr: u32, exp: u32, got: u32) {
        self.buf[self.head] = (addr, exp, got);
        self.head = (self.head + 1) % ERRLOG_CAP;
        self.len = (self.len + 1).min(ERRLOG_CAP);
        self.total = self.total.saturating_add(1);
    }
    /// Most-recent-first iterator over the retained entries.
    fn recent(&self) -> impl Iterator<Item = (u32, u32, u32)> + '_ {
        (0..self.len).map(move |k| {
            let idx = (self.head + ERRLOG_CAP - 1 - k) % ERRLOG_CAP;
            self.buf[idx]
        })
    }
}

impl Default for ErrLog {
    fn default() -> Self {
        Self::new()
    }
}

/// Overall run status. The test loops continuously (memtest-style), so the
/// live verdict comes from the cumulative error count, not a terminal state;
/// the user can pause/resume/stop/start it from the keyboard.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Waiting,
    BringUp,
    Running,
    Paused,
    Stopped,
}

/// Everything the TUI draws — updated by the test loop between chunks.
pub struct TestState {
    pub status: Status,
    pub words: usize,
    pub sdclk_mhz: u32,
    /// Full-suite loop count (1-based while running).
    pub pass: u32,
    pub elapsed_s: u32,
    pub throughput_mb_s: u32,
    /// Current group index into [`GROUPS`], the current phase label + address.
    pub cur_group: usize,
    pub phase_label: &'static str,
    pub addr: u32,
    /// Progress through the current phase, 0..=100.
    pub phase_pct: u16,
    /// Per-group cumulative error counts (this pass).
    pub group_errs: [u32; NGROUPS],
    pub errlog: ErrLog,
}

impl TestState {
    pub fn new(words: usize, sdclk_mhz: u32) -> Self {
        Self {
            status: Status::Waiting,
            words,
            sdclk_mhz,
            pass: 0,
            elapsed_s: 0,
            throughput_mb_s: 0,
            cur_group: 0,
            phase_label: "",
            addr: 0xC000_0000,
            phase_pct: 0,
            group_errs: [0; NGROUPS],
            errlog: ErrLog::new(),
        }
    }
    pub fn total_errs(&self) -> u32 {
        self.group_errs.iter().copied().sum()
    }
}

/// Write a styled string at `(x, y)`, clipped to `maxw` cells and the terminal.
fn put(buf: &mut Buffer, rows: u16, x: u16, y: u16, s: &str, style: Style, maxw: u16) {
    if y < rows {
        buf.set_stringn(x, y, s, maxw as usize, style);
    }
}

/// A horizontal progress gauge `▕███░░░▏ 54%` of total cell width `w`.
fn gauge(pct: u16, w: u16) -> String {
    let inner = w.saturating_sub(3) as usize; // brackets + a space before %
    let filled = (inner as u32 * pct.min(100) as u32 / 100) as usize;
    let mut s = String::from("\u{2595}"); // ▕
    for i in 0..inner {
        s.push(if i < filled { '\u{2588}' } else { '\u{2591}' }); // █ / ░
    }
    s.push('\u{258f}'); // ▏
    s
}

const CONTENT_W: u16 = 68;

struct MarchView<'a> {
    s: &'a TestState,
    rows: u16,
    cols: u16,
}

impl Widget for MarchView<'_> {
    fn render(self, _area: Rect, buf: &mut Buffer) {
        let rows = self.rows;
        let s = self.s;
        let ox = 1u16;

        let title = Style::default()
            .fg(Color::Black)
            .bg(Color::Cyan)
            .add_modifier(Modifier::BOLD);
        let key = Style::default().fg(Color::Gray);
        let val = Style::default().fg(Color::White);
        let dim = Style::default().fg(Color::DarkGray);
        let bar = Style::default().fg(Color::Cyan);
        let ok = Style::default()
            .fg(Color::Green)
            .add_modifier(Modifier::BOLD);
        let bad = Style::default().fg(Color::Red).add_modifier(Modifier::BOLD);
        let run = Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD);

        // --- title bar -------------------------------------------------------
        let mut bar_txt = String::from(" Daisy SDRAM March Test");
        while (bar_txt.chars().count() as u16) < CONTENT_W {
            bar_txt.push(' ');
        }
        put(buf, rows, ox, 0, &bar_txt, title, CONTENT_W);

        // --- target line -----------------------------------------------------
        let target = format!(
            "64 MiB @ 0xC0000000  {} words  SDCLK {} MHz  non-cacheable",
            s.words, s.sdclk_mhz
        );
        put(buf, rows, ox, 1, "Target ", key, 7);
        put(buf, rows, ox + 7, 1, &target, val, CONTENT_W - 7);

        // --- pass / elapsed / throughput ------------------------------------
        let mm = s.elapsed_s / 60;
        let ss = s.elapsed_s % 60;
        let stats = format!(
            "Pass {:<4}   Elapsed {:02}:{:02}   Throughput {} MB/s",
            s.pass, mm, ss, s.throughput_mb_s
        );
        put(buf, rows, ox, 2, &stats, dim, CONTENT_W);

        // --- current test + progress gauge ----------------------------------
        let group = GROUPS.get(s.cur_group).copied().unwrap_or("");
        let cur = format!(
            "[{}/{}] {}  —  {}",
            (s.cur_group + 1).min(NGROUPS),
            NGROUPS,
            group,
            s.phase_label
        );
        put(buf, rows, ox, 4, "Test  ", key, 6);
        put(buf, rows, ox + 6, 4, &cur, val, CONTENT_W - 6);
        put(buf, rows, ox, 5, "Addr  ", key, 6);
        put(buf, rows, ox + 6, 5, &format!("0x{:08X}", s.addr), val, 12);

        let g = gauge(s.phase_pct, CONTENT_W - 6);
        put(buf, rows, ox, 6, &g, bar, CONTENT_W - 6);
        put(
            buf,
            rows,
            ox + CONTENT_W - 5,
            6,
            &format!("{:>3}%", s.phase_pct.min(100)),
            val,
            4,
        );

        // --- test list -------------------------------------------------------
        put(buf, rows, ox, 8, "Tests", key, 8);
        let mut y = 9u16;
        for (gi, name) in GROUPS.iter().enumerate() {
            let errs = s.group_errs[gi];
            // Derive per-group state from progress + errors. A group is done if
            // this pass has moved past it, or a previous full pass covered it.
            let done = gi < s.cur_group || s.pass > 1;
            let (mark, mstyle) = if errs > 0 {
                ("\u{2717}", bad) // ✗
            } else if gi == s.cur_group && s.status == Status::Running {
                ("\u{25b6}", run) // ▶
            } else if done {
                ("\u{2713}", ok) // ✓
            } else {
                ("\u{00b7}", dim) // ·
            };
            put(buf, rows, ox + 2, y, mark, mstyle, 1);
            put(buf, rows, ox + 4, y, name, val, 22);
            let estr = format!("{errs} err");
            put(
                buf,
                rows,
                ox + 28,
                y,
                &estr,
                if errs > 0 { bad } else { dim },
                12,
            );
            y += 1;
        }

        // --- error log -------------------------------------------------------
        let etitle = format!("Errors ({})", s.errlog.total);
        put(buf, rows, ox, y + 1, &etitle, key, 24);
        y += 2;
        if s.errlog.total == 0 {
            put(buf, rows, ox + 2, y, "\u{2014}", dim, 2); // —
        } else {
            for (addr, exp, got) in s.errlog.recent() {
                if y >= rows {
                    break;
                }
                let bits = fault_bits(exp, got);
                let line = format!("0x{addr:08X}  exp {exp:08X} got {got:08X}  D~0x{bits:08X}");
                put(buf, rows, ox + 2, y, &line, bad, CONTENT_W - 2);
                y += 1;
            }
        }

        // --- status footer — live verdict from the cumulative error count ----
        let total = s.total_errs();
        let fy = rows.saturating_sub(3).max(y + 1);
        // The state word + a verdict badge (PASS green / FAIL red) once running.
        let (word, wstyle): (&str, Style) = match s.status {
            Status::Waiting => ("WAITING FOR TERMINAL", dim),
            Status::BringUp => ("BRINGING UP SDRAM\u{2026}", run),
            Status::Running => ("RUNNING", run),
            Status::Paused => ("PAUSED", Style::default().fg(Color::Yellow)),
            Status::Stopped => ("STOPPED", dim),
        };
        put(buf, rows, ox, fy, "Status:", key, 8);
        put(buf, rows, ox + 8, fy, word, wstyle, 22);
        if matches!(s.status, Status::Running | Status::Paused | Status::Stopped) {
            let (badge, bstyle) = if total == 0 {
                (String::from("\u{2014} PASS (no errors)"), ok)
            } else {
                (format!("\u{2014} FAIL ({total} errors)"), bad)
            };
            put(
                buf,
                rows,
                ox + 8 + word.chars().count() as u16 + 1,
                fy,
                &badge,
                bstyle,
                24,
            );
        }

        // --- controls hint ---------------------------------------------------
        let hint = match s.status {
            Status::Paused => "[space] resume    [s] stop",
            Status::Stopped => "[space] start",
            Status::Running => "[space] pause     [s] stop",
            _ => "",
        };
        put(buf, rows, ox, fy + 1, hint, dim, CONTENT_W);

        // Opaque background over the whole block so the panel overwrites any
        // banner/shell text the serial client left on screen.
        let fill_h = (fy + 1).min(rows);
        let fill_w = (ox + CONTENT_W + 1).min(self.cols);
        buf.set_style(
            Rect::new(0, 0, fill_w, fill_h),
            Style::default().bg(Color::Rgb(12, 12, 18)),
        );
    }
}

type Backend = SerialBackend<Vec<u8>>;

pub struct Tui {
    terminal: Terminal<Backend>,
    cpr: CprParser,
    cols: u16,
    rows: u16,
}

impl Tui {
    // The block is ~70×26; default big enough that a client which doesn't answer
    // a size query (picocom / macOS Terminal) still renders it whole.
    const DEFAULT_COLS: u16 = 72;
    const DEFAULT_ROWS: u16 = 28;

    pub fn new() -> Self {
        let mut backend = SerialBackend::new(Vec::new(), Self::DEFAULT_COLS, Self::DEFAULT_ROWS);
        let _ = backend.hide_cursor();
        let _ = backend.clear();
        let _ = backend.request_size();
        let terminal = Terminal::new(backend).expect("terminal init (infallible sink)");
        Self {
            terminal,
            cpr: CprParser::new(),
            cols: Self::DEFAULT_COLS,
            rows: Self::DEFAULT_ROWS,
        }
    }

    /// Call when a terminal (re)connects (DTR asserted): full repaint + re-query.
    pub fn on_connect(&mut self) {
        self.full_clear();
        let _ = self.terminal.backend_mut().request_size();
    }

    /// Hard-clear + full repaint, without re-querying size (banner mop-up).
    pub fn repaint(&mut self) {
        self.full_clear();
    }

    fn full_clear(&mut self) {
        let _ = self.terminal.clear();
        let backend = self.terminal.backend_mut();
        let _ = backend.clear();
        backend.writer_mut().extend_from_slice(b"\x1b[3J");
        let _ = backend.hide_cursor();
    }

    /// Feed host RX bytes: the CPR size reply plus any keys.
    pub fn on_input(&mut self, bytes: &[u8]) {
        if let Some((cols, rows)) = self.cpr.feed(bytes) {
            if cols > 0 && rows > 0 && (cols != self.cols || rows != self.rows) {
                self.cols = cols;
                self.rows = rows;
                self.terminal.backend_mut().resize(cols, rows);
            }
        }
    }

    pub fn render(&mut self, state: &TestState) {
        let (cols, rows) = (self.cols, self.rows);
        let _ = self.terminal.draw(|frame| {
            frame.render_widget(
                MarchView {
                    s: state,
                    rows,
                    cols,
                },
                Rect::new(0, 0, cols, rows),
            );
        });
    }

    pub fn output_pending(&mut self) -> bool {
        !self.terminal.backend_mut().writer_mut().is_empty()
    }

    pub fn drain_to<F: FnMut(&[u8]) -> Option<usize>>(&mut self, mut write: F) {
        let buf = self.terminal.backend_mut().writer_mut();
        if buf.is_empty() {
            return;
        }
        if let Some(n) = write(buf) {
            buf.drain(..n.min(buf.len()));
        }
    }
}

impl Default for Tui {
    fn default() -> Self {
        Self::new()
    }
}
