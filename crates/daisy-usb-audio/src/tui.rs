//! A demo terminal UI driven over the USB-CDC serial with `ratatui-serial`.
//!
//! HARDWARE-ONLY (needs real USB; Renode has no OTG model). Open a terminal on
//! the Daisy's CDC port (`picocom -b 115200 /dev/tty…`) and this renders a
//! status screen. The **terminal size is auto-detected** over the link: on
//! startup it issues a DSR/CPR query, and the reply (parsed by
//! `ratatui_serial::CprParser`) resizes the UI — dumb terminals that don't
//! answer just keep the default size.
//!
//! Rendering goes into an in-memory buffer (`SerialBackend<Vec<u8>>`) which the
//! main loop drains to the CDC as the host reads it — decoupling ratatui's
//! synchronous draw from the USB TX flow so a full frame never blocks the loop.

extern crate alloc;

use alloc::format;
use alloc::vec::Vec;

use ratatui_core::backend::Backend as _;
use ratatui_core::buffer::Buffer;
use ratatui_core::layout::Rect;
use ratatui_core::style::{Color, Modifier, Style};
use ratatui_core::terminal::Terminal;
use ratatui_core::widgets::Widget;
use ratatui_serial::{CprParser, SerialBackend};

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

type Backend = SerialBackend<Vec<u8>>;

/// A small demo TUI over the CDC serial.
pub struct Tui {
    terminal: Terminal<Backend>,
    cpr: CprParser,
    cols: u16,
    rows: u16,
    frames: u32,
}

impl Tui {
    /// Size assumed until the terminal reports its own.
    const DEFAULT_COLS: u16 = 80;
    const DEFAULT_ROWS: u16 = 24;

    pub fn new() -> Self {
        let mut backend = SerialBackend::new(Vec::new(), Self::DEFAULT_COLS, Self::DEFAULT_ROWS);
        // Ask the terminal for its size + hide the cursor. The reply arrives on
        // the input side (see `on_input`).
        let _ = backend.hide_cursor();
        let _ = backend.clear();
        let _ = backend.request_size();
        let terminal = Terminal::new(backend).expect("terminal init (infallible sink)");
        Self {
            terminal,
            cpr: CprParser::new(),
            cols: Self::DEFAULT_COLS,
            rows: Self::DEFAULT_ROWS,
            frames: 0,
        }
    }

    /// Feed bytes received from the host: the CPR size reply, plus any keys.
    pub fn on_input(&mut self, bytes: &[u8]) {
        if let Some((cols, rows)) = self.cpr.feed(bytes) {
            if cols > 0 && rows > 0 && (cols != self.cols || rows != self.rows) {
                self.cols = cols;
                self.rows = rows;
                // Terminal::draw autoresizes from the backend's size next frame.
                self.terminal.backend_mut().resize(cols, rows);
            }
        }
    }

    /// Render one frame into the backend's output buffer.
    pub fn render(&mut self) {
        self.frames = self.frames.wrapping_add(1);
        let (cols, rows, frames) = (self.cols, self.rows, self.frames);
        let w = cols.saturating_sub(2);
        let _ = self.terminal.draw(|frame| {
            let title = Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD);
            let dim = Style::default().fg(Color::DarkGray);
            frame.render_widget(
                Label {
                    text: "daisy-rs \u{2014} Seed 3 serial TUI",
                    style: title,
                },
                Rect::new(1, 0, w, 1),
            );
            let status = format!("terminal: {cols} x {rows}    frame {frames}");
            frame.render_widget(
                Label {
                    text: &status,
                    style: Style::default().fg(Color::Green),
                },
                Rect::new(1, 2, w, 1),
            );
            frame.render_widget(
                Label {
                    text: "size auto-detected over the serial link (DSR/CPR)",
                    style: dim,
                },
                Rect::new(1, 3, w, 1),
            );
            frame.render_widget(
                Label {
                    text: "ratatui over USB-CDC \u{2014} a host terminal is the display",
                    style: dim,
                },
                Rect::new(1, rows.saturating_sub(1), w, 1),
            );
        });
    }

    /// True while rendered bytes are still waiting to be sent — render the next
    /// frame only once this is false, so frames don't pile up unbounded.
    pub fn output_pending(&mut self) -> bool {
        !self.terminal.backend_mut().writer_mut().is_empty()
    }

    /// Push pending ANSI to the host via `write` (returns bytes actually sent),
    /// dropping what was sent. Call every loop iteration.
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
