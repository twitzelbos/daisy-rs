//! The live graphical control-panel TUI, rendered over USB-CDC with
//! `ratatui-serial` — a faithful ASCII rendering of the real Hothouse front
//! panel: the mirror-ambigram HOTHOUSE wordmark, six knobs drawn as round dials
//! with a white pointer line (2×3, like the pedal), the three toggle switches,
//! two footswitches with their LEDs, and the `>cle_` terminal-prompt mark.
//!
//! HARDWARE-ONLY (needs real USB; Renode has no OTG model). Open a terminal on
//! the Daisy's CDC port (`picocom -b 115200 /dev/tty…`) and it redraws live as
//! you move the controls. Terminal size is auto-detected over the link (DSR/CPR);
//! dumb terminals keep the default 80×24. The `SerialBackend` diffs cells between
//! frames, so a moving pot only sends the handful of bytes that changed.

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

use daisy_bsp::hothouse::ToggleswitchPosition;

/// A snapshot of every control, sampled once per frame by the main loop.
pub struct Controls {
    pub knobs: [f32; 6],
    pub toggles: [ToggleswitchPosition; 3],
    /// Footswitch held-down state.
    pub footswitches: [bool; 2],
}

/// The Hothouse wordmark, rendered like the real pedal: each letter of
/// "HOTHOUSE" is individually rotated 90° counter-clockwise, so it reads as a
/// cryptic band until you tilt your head clockwise. Letter-by-letter, rotated:
/// H→`工` O→`▢` T→`⊢` H→`工` O→`▢` U→`⊐` S→sideways-S E→`Ш`.
const LOGO: [&str; 3] = [
    "\u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588}     \u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588} \u{2588}\u{2588}\u{2588} \u{2588} \u{2588} \u{2588}",
    "  \u{2588}   \u{2588}   \u{2588} \u{2588}\u{2588}\u{2588}\u{2588}\u{2588}   \u{2588}   \u{2588}   \u{2588}     \u{2588} \u{2588} \u{2588} \u{2588} \u{2588} \u{2588} \u{2588}",
    "\u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588}     \u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588}\u{2588}\u{2588}\u{2588}\u{2588} \u{2588}\u{2588}\u{2588} \u{2588} \u{2588}\u{2588}\u{2588}\u{2588}\u{2588}",
];

/// The knob's pointer as a multi-cell line from the hub to the rim, `(row, col,
/// glyph)` within the 9×5 dial. Value 0..1 selects one of 7 over a ~270° sweep —
/// min at ~7 o'clock, clockwise over the top to max at ~5 o'clock (the pot's dead
/// zone sits at the bottom). Cells reach further horizontally than vertically
/// because terminal cells are ~2:1, which keeps the radial line looking straight.
const KLINE: [&[(u16, u16, &str)]; 7] = [
    &[(3, 3, "\u{2571}"), (3, 2, "\u{2571}")], // ╱╱  SW (min)
    &[(2, 3, "\u{2500}"), (2, 2, "\u{2500}"), (2, 1, "\u{2500}")], // ───  W
    &[(1, 3, "\u{2572}"), (1, 2, "\u{2572}")], // ╲╲  NW
    &[(1, 4, "\u{2502}")],                     // │   N
    &[(1, 5, "\u{2571}"), (1, 6, "\u{2571}")], // ╱╱  NE
    &[(2, 5, "\u{2500}"), (2, 6, "\u{2500}"), (2, 7, "\u{2500}")], // ───  E
    &[(3, 5, "\u{2572}"), (3, 6, "\u{2572}")], // ╲╲  SE (max)
];

/// Write a styled string at absolute `(x, y)`, clipped to `maxw` cells; skips
/// rows below the terminal so a short client never draws out of bounds.
fn put(buf: &mut Buffer, rows: u16, x: u16, y: u16, s: &str, style: Style, maxw: u16) {
    if y < rows {
        buf.set_stringn(x, y, s, maxw as usize, style);
    }
}

/// Centre a `len`-wide string on column `cx`.
fn cx(cx: u16, len: u16) -> u16 {
    cx.saturating_sub(len / 2)
}

/// Draws the whole panel into the frame buffer with per-cell colour. Laid out
/// like the real pedal: knobs (2×3), switches, the HOTHOUSE wordmark, then the
/// footswitches. Best viewed at ≥ 80×32; it auto-sizes and centres.
struct PanelView<'a> {
    c: &'a Controls,
    cols: u16,
    rows: u16,
}

impl Widget for PanelView<'_> {
    fn render(self, _area: Rect, buf: &mut Buffer) {
        let rows = self.rows;
        const CW: u16 = 50; // content width; the block is centred in the terminal
        let ox = self.cols.saturating_sub(CW) / 2;
        let colx = [ox + 2, ox + 20, ox + 38]; // knob left edges (9 wide), aligned

        let logo = Style::default()
            .fg(Color::White)
            .add_modifier(Modifier::BOLD);
        let sub = Style::default().fg(Color::DarkGray);
        let bezel = Style::default().fg(Color::DarkGray);
        let ptr = Style::default()
            .fg(Color::White)
            .add_modifier(Modifier::BOLD);
        let lab = Style::default().fg(Color::Gray);
        let pct = Style::default().fg(Color::Green);
        let sw = Style::default().fg(Color::Yellow);
        let led_on = Style::default().fg(Color::Red).add_modifier(Modifier::BOLD);
        let led_off = Style::default().fg(Color::DarkGray);
        let btn = Style::default().fg(Color::Gray);
        let cle = Style::default()
            .fg(Color::Green)
            .add_modifier(Modifier::BOLD);

        // --- six knobs, 2×3, big round dials with a pointer line + name / % ----
        let mut y = 1u16; // leave row 0 for the enclosure's top edge
        for row in 0..2u16 {
            let ky = y;
            for (col, &x) in colx.iter().enumerate() {
                let idx = row as usize * 3 + col;
                put(
                    buf,
                    rows,
                    x,
                    ky,
                    "\u{256d}\u{2500}\u{2500}\u{2500}\u{2500}\u{2500}\u{2500}\u{2500}\u{256e}",
                    bezel,
                    9,
                ); // ╭───────╮
                put(buf, rows, x, ky + 1, "\u{2502}       \u{2502}", bezel, 9); // │       │
                put(
                    buf,
                    rows,
                    x,
                    ky + 2,
                    "\u{2502}   \u{25cf}   \u{2502}",
                    bezel,
                    9,
                ); // │   ●   │  (hub)
                put(buf, rows, x, ky + 3, "\u{2502}       \u{2502}", bezel, 9); // │       │
                put(
                    buf,
                    rows,
                    x,
                    ky + 4,
                    "\u{2570}\u{2500}\u{2500}\u{2500}\u{2500}\u{2500}\u{2500}\u{2500}\u{256f}",
                    bezel,
                    9,
                ); // ╰───────╯
                let v = self.c.knobs[idx].clamp(0.0, 1.0);
                for &(pr, pc, glyph) in KLINE[((v * 6.0 + 0.5) as usize).min(6)] {
                    put(buf, rows, x + pc, ky + pr, glyph, ptr, 1);
                }
                let name = format!("KNOB {}", idx + 1);
                put(
                    buf,
                    rows,
                    cx(x + 4, name.len() as u16),
                    ky + 5,
                    &name,
                    lab,
                    8,
                );
                let p = format!("{:.0}%", v * 100.0);
                put(buf, rows, cx(x + 4, p.len() as u16), ky + 6, &p, pct, 5);
            }
            y += 8;
        }

        // --- three toggle switches, aligned under the knob columns ------------
        for (col, &x) in colx.iter().enumerate() {
            let g = match self.c.toggles[col] {
                ToggleswitchPosition::Up => "[\u{25b2}]",     // [▲]
                ToggleswitchPosition::Middle => "[\u{2550}]", // [═]
                ToggleswitchPosition::Down => "[\u{25bc}]",   // [▼]
            };
            put(buf, rows, cx(x + 4, 3), y, g, sw, 3);
            let s = format!("SWITCH {}", col + 1);
            put(buf, rows, cx(x + 4, s.len() as u16), y + 1, &s, lab, 9);
        }
        y += 3;

        // --- HOTHOUSE wordmark + tagline (between switches and footswitches) --
        for line in LOGO {
            put(
                buf,
                rows,
                cx(ox + CW / 2, line.chars().count() as u16),
                y,
                line,
                logo,
                CW,
            );
            y += 1;
        }
        let tag = "diy :: dsp :: platform";
        put(
            buf,
            rows,
            cx(ox + CW / 2, tag.len() as u16),
            y,
            tag,
            sub,
            CW,
        );
        y += 2;

        // --- footswitches + LEDs + the `>cle_` mark ---------------------------
        // Aligned so each footswitch centres under the outer knob/switch columns
        // (knob centres are at ox+6 and ox+42; the 7-wide box centres on x+3).
        let fx = [ox + 3, ox + 39];
        for (i, &x) in fx.iter().enumerate() {
            let on = self.c.footswitches[i];
            put(
                buf,
                rows,
                x + 3,
                y,
                "\u{25cf}",
                if on { led_on } else { led_off },
                1,
            ); // ● LED
            put(
                buf,
                rows,
                x,
                y + 1,
                "\u{256d}\u{2500}\u{2500}\u{2500}\u{2500}\u{2500}\u{256e}",
                btn,
                7,
            ); // ╭─────╮
            let face = if on {
                "\u{2502} \u{2588}\u{2588}\u{2588} \u{2502}" // │ ███ │ (stomped)
            } else {
                "\u{2502} \u{2593}\u{2593}\u{2593} \u{2502}" // │ ▓▓▓ │
            };
            put(buf, rows, x, y + 2, face, btn, 7);
            put(
                buf,
                rows,
                x,
                y + 3,
                "\u{2570}\u{2500}\u{2500}\u{2500}\u{2500}\u{2500}\u{256f}",
                btn,
                7,
            ); // ╰─────╯
            let fl = format!("FOOTSWITCH {}", i + 1);
            put(buf, rows, cx(x + 3, fl.len() as u16), y + 4, &fl, lab, 12);
        }
        put(buf, rows, cx(ox + CW / 2, 5), y + 2, ">cle_", cle, 6); // terminal-prompt brand

        // --- enclosure (the white pedal body) ---------------------------------
        let bottom = y + 4;
        let edge = Style::default().fg(Color::Gray);
        let bx0 = ox.saturating_sub(2);
        let bx1 = ox + CW + 1;
        let mut top = String::from("\u{256d}"); // ╭
        let mut bot = String::from("\u{2570}"); // ╰
        for _ in bx0 + 1..bx1 {
            top.push('\u{2500}'); // ─
            bot.push('\u{2500}');
        }
        top.push('\u{256e}'); // ╮
        bot.push('\u{256f}'); // ╯
        put(buf, rows, bx0, 0, &top, edge, bx1 - bx0 + 1);
        put(buf, rows, bx0, bottom + 1, &bot, edge, bx1 - bx0 + 1);
        for yy in 1..=bottom {
            put(buf, rows, bx0, yy, "\u{2502}", edge, 1); // │
            put(buf, rows, bx1, yy, "\u{2502}", edge, 1);
        }
    }
}

type Backend = SerialBackend<Vec<u8>>;

pub struct Panel {
    terminal: Terminal<Backend>,
    cpr: CprParser,
    cols: u16,
    rows: u16,
}

impl Panel {
    const DEFAULT_COLS: u16 = 80;
    const DEFAULT_ROWS: u16 = 24;

    pub fn new() -> Self {
        let mut backend = SerialBackend::new(Vec::new(), Self::DEFAULT_COLS, Self::DEFAULT_ROWS);
        let _ = backend.hide_cursor();
        let _ = backend.clear();
        let _ = backend.request_size(); // reply arrives on the input side
        let terminal = Terminal::new(backend).expect("terminal init (infallible sink)");
        Self {
            terminal,
            cpr: CprParser::new(),
            cols: Self::DEFAULT_COLS,
            rows: Self::DEFAULT_ROWS,
        }
    }

    /// Call when a terminal (re)connects (DTR asserted). ratatui only sends cell
    /// *diffs* after the first frame, so a client that attaches mid-run sees a
    /// blank screen. Wipe the client's screen, hide its cursor, re-query its
    /// size, and force the next frame to repaint in FULL.
    pub fn on_connect(&mut self) {
        self.full_clear();
        // Ask the client its size; its CPR reply also tells us it's now listening
        // (see `on_input`), which is when we clear again to beat the startup race.
        let _ = self.terminal.backend_mut().request_size();
    }

    /// Hard-wipe the client screen + reset ratatui's diff baseline so the next
    /// frame repaints in FULL (over a cleared screen).
    fn full_clear(&mut self) {
        let _ = self.terminal.clear(); // reset the cell-diff baseline
        let backend = self.terminal.backend_mut();
        let _ = backend.clear(); // ESC[2J + home
        backend.writer_mut().extend_from_slice(b"\x1b[3J"); // drop scrollback too
        let _ = backend.hide_cursor();
    }

    /// Feed bytes received from the host: the CPR size reply plus any keys.
    pub fn on_input(&mut self, bytes: &[u8]) {
        if let Some((cols, rows)) = self.cpr.feed(bytes) {
            if cols > 0 && rows > 0 {
                if cols != self.cols || rows != self.rows {
                    self.cols = cols;
                    self.rows = rows;
                    self.terminal.backend_mut().resize(cols, rows);
                }
                // The client answered our size query, so it is now attached and
                // reading. An `ESC[2J` sent at DTR-assert is often swallowed while
                // the client (e.g. picocom) is still starting up, leaving stale
                // shell text around the panel — so re-clear now that we KNOW the
                // terminal is listening, and repaint in full.
                self.full_clear();
            }
        }
    }

    /// Draw one frame from the current control snapshot.
    pub fn render(&mut self, c: &Controls) {
        let (cols, rows) = (self.cols, self.rows);
        let _ = self.terminal.draw(|frame| {
            frame.render_widget(PanelView { c, cols, rows }, Rect::new(0, 0, cols, rows));
        });
    }

    /// True while a rendered frame is still waiting to be sent.
    pub fn output_pending(&mut self) -> bool {
        !self.terminal.backend_mut().writer_mut().is_empty()
    }

    /// Push pending ANSI to the host, dropping what was accepted.
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

impl Default for Panel {
    fn default() -> Self {
        Self::new()
    }
}
