//! The live control-panel TUI, rendered over USB-CDC with `ratatui-serial`.
//!
//! HARDWARE-ONLY (needs real USB; Renode has no OTG model). Open a terminal on
//! the Daisy's CDC port (`picocom -b 115200 /dev/tty…`) and this draws the
//! Hothouse front panel — six pot bars, three toggle positions, two footswitch
//! indicators — refreshed live as you move the controls. Terminal size is
//! auto-detected over the link (DSR/CPR); dumb terminals keep the default 80×24.
//!
//! The `SerialBackend` diffs cells between frames, so a live redraw only sends
//! the characters that actually changed — a moving pot is a handful of bytes.

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

/// ratatui-core ships the `Widget` trait but no concrete widgets; this is the
/// one-line label we need (a widget gets `&mut Buffer`, so it can `set_string`).
struct Label<'a> {
    text: &'a str,
    style: Style,
}

impl Widget for Label<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        buf.set_stringn(area.x, area.y, self.text, area.width as usize, self.style);
    }
}

/// A horizontal 0.0..1.0 bar built from block glyphs, `width` cells wide.
fn bar(value: f32, width: usize) -> String {
    let v = value.clamp(0.0, 1.0);
    // No `f32::round` in no_std (no libm); v >= 0 so +0.5 truncation rounds.
    let filled = ((v * width as f32 + 0.5) as usize).min(width);
    let mut s = String::with_capacity(width * 3);
    for i in 0..width {
        s.push(if i < filled { '\u{2588}' } else { '\u{2591}' }); // █ / ░
    }
    s
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

fn toggle_str(p: ToggleswitchPosition) -> &'static str {
    match p {
        ToggleswitchPosition::Up => "\u{2191} UP  ",     // ↑ UP
        ToggleswitchPosition::Middle => "\u{2014} MID ", // — MID
        ToggleswitchPosition::Down => "\u{2193} DOWN",   // ↓ DOWN
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
    const BAR_WIDTH: usize = 24;

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

    /// Call when a terminal (re)connects (DTR asserted). ratatui only sends
    /// cell *diffs* after the first frame, so a `screen`/`picocom` client that
    /// attaches mid-run sees a blank screen. Wipe the client's screen, hide its
    /// cursor, re-query its size, and force the next frame to repaint in FULL.
    pub fn on_connect(&mut self) {
        // Reset ratatui's cell diff baseline so the next frame repaints in FULL.
        let _ = self.terminal.clear();
        // Then HARD-wipe the client's screen: `Terminal::clear` only guarantees
        // the diff reset, so stale shell/terminal output around the panel would
        // otherwise survive (it only gets overwritten where cells are drawn).
        // `backend.clear()` emits ESC[2J + homes the cursor + resyncs our cursor
        // tracking; ESC[3J drops the scrollback so it can't be scrolled back in.
        let backend = self.terminal.backend_mut();
        let _ = backend.clear();
        backend.writer_mut().extend_from_slice(b"\x1b[3J");
        let _ = backend.hide_cursor();
        let _ = backend.request_size();
    }

    /// Feed bytes received from the host: the CPR size reply plus any keys.
    pub fn on_input(&mut self, bytes: &[u8]) {
        if let Some((cols, rows)) = self.cpr.feed(bytes) {
            if cols > 0 && rows > 0 && (cols != self.cols || rows != self.rows) {
                self.cols = cols;
                self.rows = rows;
                self.terminal.backend_mut().resize(cols, rows);
            }
        }
    }

    /// Draw one frame from the current control snapshot.
    pub fn render(&mut self, c: &Controls) {
        let (cols, rows) = (self.cols, self.rows);
        let w = cols.saturating_sub(2);
        let title = Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD);
        let dim = Style::default().fg(Color::DarkGray);
        let knob_style = Style::default().fg(Color::Green);
        let tog_style = Style::default().fg(Color::Yellow);

        // Warm off-white for the wordmark, echoing the white pedal.
        let logo_style = Style::default()
            .fg(Color::Gray)
            .add_modifier(Modifier::BOLD);

        let _ = self.terminal.draw(|frame| {
            // Rotated HOTHOUSE wordmark (rows 0-2), centred in the width.
            for (i, line) in LOGO.iter().enumerate() {
                let pad = (w as usize).saturating_sub(line.chars().count()) / 2;
                frame.render_widget(
                    Label {
                        text: line,
                        style: logo_style,
                    },
                    Rect::new(1 + pad as u16, i as u16, w, 1),
                );
            }

            frame.render_widget(
                Label {
                    text: "daisy-rs \u{2014} Hothouse control panel",
                    style: title,
                },
                Rect::new(1, 3, w, 1),
            );
            let sub = format!("live over USB-CDC \u{00b7} terminal {cols}\u{00d7}{rows}");
            frame.render_widget(
                Label {
                    text: &sub,
                    style: dim,
                },
                Rect::new(1, 4, w, 1),
            );

            // Six knobs.
            for (i, &v) in c.knobs.iter().enumerate() {
                let line = format!(
                    "Knob {}  [{}] {:3.0}%",
                    i + 1,
                    bar(v, Self::BAR_WIDTH),
                    v * 100.0
                );
                frame.render_widget(
                    Label {
                        text: &line,
                        style: knob_style,
                    },
                    Rect::new(1, 6 + i as u16, w, 1),
                );
            }

            // Three toggles.
            for (i, &p) in c.toggles.iter().enumerate() {
                let line = format!("Toggle {}   {}", i + 1, toggle_str(p));
                frame.render_widget(
                    Label {
                        text: &line,
                        style: tog_style,
                    },
                    Rect::new(1, 13 + i as u16, w, 1),
                );
            }

            // Two footswitches.
            for (i, &pressed) in c.footswitches.iter().enumerate() {
                let (glyph, word, color) = if pressed {
                    ('\u{25cf}', "PRESSED", Color::Red) // ●
                } else {
                    ('\u{25cb}', "\u{2014}", Color::DarkGray) // ○ —
                };
                let line = format!("Footswitch {}  {} {}", i + 1, glyph, word);
                frame.render_widget(
                    Label {
                        text: &line,
                        style: Style::default().fg(color),
                    },
                    Rect::new(1, 17 + i as u16, w, 1),
                );
            }

            frame.render_widget(
                Label {
                    text: "move a control \u{2014} the panel updates live",
                    style: dim,
                },
                Rect::new(1, rows.saturating_sub(1), w, 1),
            );
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
