#![cfg_attr(not(test), no_std)]

//! A [`ratatui`](https://ratatui.rs) [`Backend`] that renders to an arbitrary
//! byte sink using ANSI / VT100 escape sequences, instead of a host OS terminal.
//!
//! This lets an embedded device (e.g. a Daisy Seed) *host* a full terminal UI
//! over a **USB-CDC / serial** link: the device drives the UI and a terminal
//! emulator on the host (`picocom`, `screen`, PuTTY, …) is the display. It's
//! `no_std` — the only requirement is a global allocator, which `ratatui-core`
//! needs (this crate itself allocates nothing).
//!
//! ```ignore
//! use ratatui_serial::{SerialBackend, Writer};
//! use ratatui_core::terminal::Terminal;
//!
//! // `port` is your serial peripheral wrapped in a `Writer` impl (or, with the
//! // `embedded-io` feature, anything implementing `embedded_io::Write`).
//! let backend = SerialBackend::new(port, 80, 24);
//! let mut terminal = Terminal::new(backend)?;
//! terminal.draw(|frame| { /* ratatui widgets */ })?;
//! ```
//!
//! The backend diffs cursor position and SGR style between cells so a redraw
//! emits the minimum escape traffic. Terminal size is supplied by the caller
//! (there is no `SIGWINCH` over a serial link); call [`SerialBackend::resize`]
//! if you query it out of band.

extern crate alloc;

use core::fmt;

use ratatui_core::backend::{Backend, ClearType, WindowSize};
use ratatui_core::buffer::Cell;
use ratatui_core::layout::{Position, Size};
use ratatui_core::style::{Color, Modifier};

/// A byte sink the backend writes ANSI to. Implement this for your serial port
/// (or enable the `embedded-io` feature to use any `embedded_io::Write`).
pub trait Writer {
    /// The error returned by the underlying transport.
    type Error;
    /// Write every byte, or fail.
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), Self::Error>;
    /// Flush any buffered bytes to the transport. Defaults to a no-op.
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Buffer ANSI in memory (host tests, or batch-then-send). Infallible.
///
/// Disabled when the `embedded-io` feature is on, where the blanket impl below
/// covers any `embedded_io::Write` (which would overlap this one).
#[cfg(not(feature = "embedded-io"))]
impl Writer for alloc::vec::Vec<u8> {
    type Error = core::convert::Infallible;
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.extend_from_slice(bytes);
        Ok(())
    }
}

#[cfg(feature = "embedded-io")]
impl<W: embedded_io::Write> Writer for W {
    type Error = W::Error;
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        embedded_io::Write::write_all(self, bytes)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(self)
    }
}

/// Wraps the transport's error so the backend can satisfy `Backend::Error`.
#[derive(Debug)]
pub struct Error<E>(pub E);

impl<E: fmt::Debug> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "serial-backend transport error: {:?}", self.0)
    }
}

impl<E: fmt::Debug> core::error::Error for Error<E> {}

/// Control Sequence Introducer.
const CSI: &[u8] = b"\x1b[";

/// A ratatui backend that renders to a byte sink as an ANSI/VT100 terminal.
pub struct SerialBackend<W> {
    w: W,
    size: Size,
    cursor: Position,
    // Last SGR state we emitted, to diff styles across cells.
    fg: Color,
    bg: Color,
    modifier: Modifier,
    style_valid: bool,
}

impl<W> SerialBackend<W> {
    /// Create a backend for a `columns` × `rows` terminal over `writer`.
    pub fn new(writer: W, columns: u16, rows: u16) -> Self {
        Self {
            w: writer,
            size: Size {
                width: columns,
                height: rows,
            },
            cursor: Position { x: 0, y: 0 },
            fg: Color::Reset,
            bg: Color::Reset,
            modifier: Modifier::empty(),
            style_valid: false,
        }
    }

    /// Update the terminal size (e.g. after querying it out of band).
    pub fn resize(&mut self, columns: u16, rows: u16) {
        self.size = Size {
            width: columns,
            height: rows,
        };
    }

    /// Borrow the underlying writer.
    pub fn writer_mut(&mut self) -> &mut W {
        &mut self.w
    }

    /// Consume the backend and return the underlying writer.
    pub fn into_writer(self) -> W {
        self.w
    }
}

impl<W: Writer> SerialBackend<W> {
    #[inline]
    fn raw(&mut self, bytes: &[u8]) -> Result<(), Error<W::Error>> {
        self.w.write_all(bytes).map_err(Error)
    }

    /// Write a `u16` as decimal ASCII (max 5 digits, no allocation).
    fn num(&mut self, n: u16) -> Result<(), Error<W::Error>> {
        let mut buf = [0u8; 5];
        let mut i = buf.len();
        let mut v = n;
        loop {
            i -= 1;
            buf[i] = b'0' + (v % 10) as u8;
            v /= 10;
            if v == 0 {
                break;
            }
        }
        self.raw(&buf[i..])
    }

    /// Emit a leading `;` then a numeric SGR/param.
    fn param(&mut self, n: u16) -> Result<(), Error<W::Error>> {
        self.raw(b";")?;
        self.num(n)
    }

    /// `CSI y+1 ; x+1 H` — absolute cursor move (VT coords are 1-based).
    fn move_to(&mut self, x: u16, y: u16) -> Result<(), Error<W::Error>> {
        self.raw(CSI)?;
        self.num(y.saturating_add(1))?;
        self.raw(b";")?;
        self.num(x.saturating_add(1))?;
        self.raw(b"H")?;
        self.cursor = Position { x, y };
        Ok(())
    }

    fn color_params(&mut self, c: Color, foreground: bool) -> Result<(), Error<W::Error>> {
        let base: u16 = if foreground { 30 } else { 40 };
        let bright: u16 = if foreground { 90 } else { 100 };
        match c {
            // We always emit a full `CSI 0` reset before re-applying the style,
            // so the default fg/bg (39/49) are already active — emit nothing.
            Color::Reset => Ok(()),
            Color::Black => self.param(base),
            Color::Red => self.param(base + 1),
            Color::Green => self.param(base + 2),
            Color::Yellow => self.param(base + 3),
            Color::Blue => self.param(base + 4),
            Color::Magenta => self.param(base + 5),
            Color::Cyan => self.param(base + 6),
            Color::Gray => self.param(base + 7),
            Color::DarkGray => self.param(bright),
            Color::LightRed => self.param(bright + 1),
            Color::LightGreen => self.param(bright + 2),
            Color::LightYellow => self.param(bright + 3),
            Color::LightBlue => self.param(bright + 4),
            Color::LightMagenta => self.param(bright + 5),
            Color::LightCyan => self.param(bright + 6),
            Color::White => self.param(bright + 7),
            Color::Indexed(i) => {
                self.param(if foreground { 38 } else { 48 })?;
                self.raw(b";5;")?;
                self.num(i as u16)
            }
            Color::Rgb(r, g, b) => {
                self.param(if foreground { 38 } else { 48 })?;
                self.raw(b";2;")?;
                self.num(r as u16)?;
                self.raw(b";")?;
                self.num(g as u16)?;
                self.raw(b";")?;
                self.num(b as u16)
            }
        }
    }

    fn modifier_params(&mut self, m: Modifier) -> Result<(), Error<W::Error>> {
        // SGR codes: bold 1, dim 2, italic 3, underline 4, slow-blink 5,
        // rapid-blink 6, reverse 7, hidden 8, crossed-out 9.
        for (flag, code) in [
            (Modifier::BOLD, 1u16),
            (Modifier::DIM, 2),
            (Modifier::ITALIC, 3),
            (Modifier::UNDERLINED, 4),
            (Modifier::SLOW_BLINK, 5),
            (Modifier::RAPID_BLINK, 6),
            (Modifier::REVERSED, 7),
            (Modifier::HIDDEN, 8),
            (Modifier::CROSSED_OUT, 9),
        ] {
            if m.contains(flag) {
                self.param(code)?;
            }
        }
        Ok(())
    }

    /// Emit one combined SGR (`CSI 0 ; … m`) only when the style changed.
    fn apply_style(&mut self, cell: &Cell) -> Result<(), Error<W::Error>> {
        if self.style_valid
            && cell.fg == self.fg
            && cell.bg == self.bg
            && cell.modifier == self.modifier
        {
            return Ok(());
        }
        self.raw(CSI)?;
        self.raw(b"0")?; // reset, then re-apply the whole style
        self.modifier_params(cell.modifier)?;
        self.color_params(cell.fg, true)?;
        self.color_params(cell.bg, false)?;
        self.raw(b"m")?;
        self.fg = cell.fg;
        self.bg = cell.bg;
        self.modifier = cell.modifier;
        self.style_valid = true;
        Ok(())
    }

    /// Ask the terminal to report its size. The reply arrives asynchronously on
    /// the **input** side as a Cursor-Position-Report (`CSI rows ; cols R`) —
    /// feed the received bytes to a [`CprParser`] and call [`Self::resize`] with
    /// the result.
    ///
    /// The sequence saves the cursor, moves it to an impossibly far corner
    /// (which the terminal clamps to its actual bottom-right), issues a Device
    /// Status Report query, then restores the cursor — so it's safe to call at
    /// any time, including mid-frame. Terminals that don't answer DSR simply
    /// leave your default size in place.
    pub fn request_size(&mut self) -> Result<(), Error<W::Error>> {
        // DECSC, CUP 999;999, DSR-CPR, DECRC.
        self.raw(b"\x1b7\x1b[999;999H\x1b[6n\x1b8")?;
        // The save/restore leaves the cursor where it was, but we don't know
        // where that is relative to our tracking — force the next draw to move.
        self.cursor = Position {
            x: u16::MAX,
            y: u16::MAX,
        };
        Ok(())
    }
}

/// Parses Cursor-Position-Report replies (`CSI rows ; cols R`) out of an input
/// byte stream — e.g. the terminal's answer to [`SerialBackend::request_size`],
/// possibly interleaved with keystrokes. Feed it received bytes; it yields
/// `(columns, rows)` when a full report completes.
///
/// ```
/// # use ratatui_serial::CprParser;
/// let mut p = CprParser::new();
/// assert_eq!(p.feed(b"\x1b[24;80R"), Some((80, 24)));
/// ```
#[derive(Clone, Copy, Default)]
pub struct CprParser {
    state: CprState,
    rows: u16,
    cols: u16,
}

#[derive(Clone, Copy, Default, PartialEq)]
enum CprState {
    #[default]
    Idle,
    Esc,
    Bracket,
    Rows,
    Semi,
    Cols,
}

impl CprParser {
    /// A fresh parser in the idle state.
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one byte. Returns `Some((columns, rows))` when a report completes.
    pub fn push(&mut self, b: u8) -> Option<(u16, u16)> {
        const ESC: u8 = 0x1b;
        match self.state {
            CprState::Idle => {
                if b == ESC {
                    self.state = CprState::Esc;
                }
            }
            CprState::Esc => {
                self.state = if b == b'[' {
                    CprState::Bracket
                } else if b == ESC {
                    CprState::Esc
                } else {
                    CprState::Idle
                };
            }
            CprState::Bracket => {
                if b.is_ascii_digit() {
                    self.rows = (b - b'0') as u16;
                    self.state = CprState::Rows;
                } else {
                    self.restart(b);
                }
            }
            CprState::Rows => {
                if b.is_ascii_digit() {
                    self.rows = self
                        .rows
                        .saturating_mul(10)
                        .saturating_add((b - b'0') as u16);
                } else if b == b';' {
                    self.cols = 0;
                    self.state = CprState::Semi;
                } else {
                    self.restart(b);
                }
            }
            CprState::Semi => {
                if b.is_ascii_digit() {
                    self.cols = (b - b'0') as u16;
                    self.state = CprState::Cols;
                } else {
                    self.restart(b);
                }
            }
            CprState::Cols => {
                if b.is_ascii_digit() {
                    self.cols = self
                        .cols
                        .saturating_mul(10)
                        .saturating_add((b - b'0') as u16);
                } else if b == b'R' {
                    let result = (self.cols, self.rows);
                    self.state = CprState::Idle;
                    return Some(result);
                } else {
                    self.restart(b);
                }
            }
        }
        None
    }

    /// Feed a slice; returns the **last** completed report seen, if any.
    pub fn feed(&mut self, bytes: &[u8]) -> Option<(u16, u16)> {
        let mut last = None;
        for &b in bytes {
            if let Some(size) = self.push(b) {
                last = Some(size);
            }
        }
        last
    }

    /// A stray byte broke the sequence: re-sync (an `ESC` starts a new one).
    fn restart(&mut self, b: u8) {
        self.state = if b == 0x1b {
            CprState::Esc
        } else {
            CprState::Idle
        };
    }
}

impl<W: Writer> Backend for SerialBackend<W>
where
    W::Error: fmt::Debug,
{
    type Error = Error<W::Error>;

    fn draw<'a, I>(&mut self, content: I) -> Result<(), Self::Error>
    where
        I: Iterator<Item = (u16, u16, &'a Cell)>,
    {
        // `content` is the diff ratatui already filtered (skip cells excluded),
        // so every cell here should be rendered.
        for (x, y, cell) in content {
            if self.cursor.x != x || self.cursor.y != y {
                self.move_to(x, y)?;
            }
            self.apply_style(cell)?;
            self.raw(cell.symbol().as_bytes())?;
            // The terminal advances one column after printing (wide graphemes
            // occupy the next cell which ratatui marks `skip`).
            self.cursor.x = x.saturating_add(1);
            self.cursor.y = y;
        }
        Ok(())
    }

    fn hide_cursor(&mut self) -> Result<(), Self::Error> {
        self.raw(CSI)?;
        self.raw(b"?25l")
    }

    fn show_cursor(&mut self) -> Result<(), Self::Error> {
        self.raw(CSI)?;
        self.raw(b"?25h")
    }

    fn get_cursor_position(&mut self) -> Result<Position, Self::Error> {
        // A serial link can't be queried synchronously; report the tracked position.
        Ok(self.cursor)
    }

    fn set_cursor_position<P: Into<Position>>(&mut self, position: P) -> Result<(), Self::Error> {
        let p = position.into();
        self.move_to(p.x, p.y)
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        self.raw(CSI)?;
        self.raw(b"2J")?;
        self.move_to(0, 0)?;
        self.style_valid = false;
        Ok(())
    }

    fn clear_region(&mut self, clear_type: ClearType) -> Result<(), Self::Error> {
        self.raw(CSI)?;
        match clear_type {
            ClearType::All => self.raw(b"2J"),
            ClearType::AfterCursor => self.raw(b"0J"),
            ClearType::BeforeCursor => self.raw(b"1J"),
            ClearType::CurrentLine => self.raw(b"2K"),
            ClearType::UntilNewLine => self.raw(b"0K"),
        }
    }

    fn size(&self) -> Result<Size, Self::Error> {
        Ok(self.size)
    }

    fn window_size(&mut self) -> Result<WindowSize, Self::Error> {
        Ok(WindowSize {
            columns_rows: self.size,
            pixels: Size {
                width: 0,
                height: 0,
            },
        })
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.w.flush().map_err(Error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::String;
    use alloc::vec::Vec;

    fn cell(symbol: &str, fg: Color, modifier: Modifier) -> Cell {
        let mut c = Cell::default();
        c.set_symbol(symbol);
        c.fg = fg;
        c.modifier = modifier;
        c
    }

    fn render<F: FnOnce(&mut SerialBackend<Vec<u8>>)>(f: F) -> String {
        let mut backend = SerialBackend::new(Vec::new(), 80, 24);
        f(&mut backend);
        String::from_utf8(backend.into_writer()).unwrap()
    }

    #[test]
    fn draws_positioned_styled_text() {
        let a = cell("H", Color::Green, Modifier::BOLD);
        let b = cell("i", Color::Green, Modifier::BOLD);
        let out = render(|be| {
            be.draw([(2u16, 1u16, &a), (3u16, 1u16, &b)].into_iter())
                .unwrap();
        });
        // Cursor move to (2,1) → 1-based row;col = 2;3.
        assert!(out.contains("\x1b[2;3H"), "cursor move missing: {out:?}");
        // One combined SGR with reset(0), bold(1), green fg(32).
        assert!(out.contains("\x1b[0;1;32m"), "style SGR missing: {out:?}");
        assert!(out.contains("Hi"), "content missing: {out:?}");
        // Adjacent second cell must NOT re-emit a move or a style.
        assert_eq!(out.matches('H').count(), 2); // the two 'H' are the CUP 'H' + text 'H'
        assert_eq!(out.matches("\x1b[0;1;32m").count(), 1);
    }

    #[test]
    fn cursor_clear_and_size() {
        let out = render(|be| {
            be.hide_cursor().unwrap();
            be.set_cursor_position(Position { x: 5, y: 3 }).unwrap();
            assert_eq!(be.get_cursor_position().unwrap(), Position { x: 5, y: 3 });
            assert_eq!(
                be.size().unwrap(),
                Size {
                    width: 80,
                    height: 24
                }
            );
            be.clear().unwrap();
            be.show_cursor().unwrap();
        });
        assert!(out.contains("\x1b[?25l"), "hide-cursor missing: {out:?}");
        assert!(out.contains("\x1b[4;6H"), "cursor move missing: {out:?}"); // (5,3) → 4;6
        assert!(out.contains("\x1b[2J"), "clear missing: {out:?}");
        assert!(out.contains("\x1b[?25h"), "show-cursor missing: {out:?}");
    }

    #[test]
    fn rgb_and_indexed_colors() {
        let a = cell("x", Color::Rgb(10, 20, 30), Modifier::empty());
        let b = cell("y", Color::Indexed(200), Modifier::empty());
        let out = render(|be| {
            be.draw([(0u16, 0u16, &a)].into_iter()).unwrap();
            be.draw([(0u16, 1u16, &b)].into_iter()).unwrap();
        });
        assert!(
            out.contains("\x1b[0;38;2;10;20;30m"),
            "rgb SGR missing: {out:?}"
        );
        assert!(
            out.contains("\x1b[0;38;5;200m"),
            "indexed SGR missing: {out:?}"
        );
    }

    #[test]
    fn request_size_emits_dsr_query() {
        let out = render(|be| be.request_size().unwrap());
        assert!(
            out.contains("\x1b[999;999H"),
            "far-corner move missing: {out:?}"
        );
        assert!(out.contains("\x1b[6n"), "DSR query missing: {out:?}");
    }

    #[test]
    fn cpr_parser_basic() {
        let mut p = CprParser::new();
        assert_eq!(p.feed(b"\x1b[24;80R"), Some((80, 24)));
    }

    #[test]
    fn cpr_parser_byte_by_byte_multidigit() {
        let mut p = CprParser::new();
        let mut got = None;
        for &b in b"\x1b[135;238R" {
            if let Some(s) = p.push(b) {
                got = Some(s);
            }
        }
        assert_eq!(got, Some((238, 135))); // (columns, rows)
    }

    #[test]
    fn cpr_parser_ignores_garbage_and_resyncs() {
        let mut p = CprParser::new();
        assert_eq!(p.feed(b"qwe\x1b[X"), None); // keystrokes + a false start
        assert_eq!(p.feed(b"\x1b[2;3R"), Some((3, 2))); // real report afterwards
    }

    #[test]
    fn cpr_parser_restarts_on_nested_esc() {
        let mut p = CprParser::new();
        assert_eq!(p.feed(b"\x1b[12\x1b[24;80R"), Some((80, 24)));
    }
}
