//! The ISO 10303-21 exchange structure: a file's text to its header and
//! its instances, with nothing resolved (ADR-0025 §3). What an instance
//! *means* — which entity, which references are legal, what a parameter
//! is in which unit — is the reader's; this module only says what the
//! file holds, and where it stops being Part 21.
//!
//! Read: the header's entities (`FILE_DESCRIPTION`, `FILE_NAME` and
//! `FILE_SCHEMA` are required; any other is kept), one or more `DATA`
//! sections, a named one included, simple and complex (external-mapping)
//! instances, and every parameter of the grammar — integers, reals in
//! every spelling it allows (`1.`, `-1.E-5`), strings with `''` and the
//! `\S\`, `\X\`, `\X2\…\X0\` and `\X4\…\X0\` encodings, enumerations,
//! binaries, typed parameters, references, lists, `$` and `*` — with
//! `/* comments */` anywhere a space may be. Keywords are read as written
//! and upper-cased.
//!
//! Not read, and an error naming it: an edition-3 `ANCHOR`, `REFERENCE`
//! or `SIGNATURE` section and a value instance (`@12`), which the B-Rep
//! subset never needs.
//!
//! Guarantees: never panics on any input, and terminates in time linear
//! in its length; instances come back in a `BTreeMap` by id, so every walk
//! over them is deterministic; the first error stops the parse and names
//! the line and column (both from 1, the column in characters) and the
//! instance it was inside, if any.

use core::fmt;
use std::collections::BTreeMap;

/// A parsed exchange structure: its header, and its instances by id
/// across every `DATA` section.
///
/// ```
/// use arris_io::step::part21::{self, Param};
///
/// let text = "ISO-10303-21;
/// HEADER;
/// FILE_DESCRIPTION(('a point'),'2;1');
/// FILE_NAME('p.stp','',(''),(''),'','','');
/// FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
/// ENDSEC;
/// DATA;
/// #1=CARTESIAN_POINT('',(1.,-2.5E-1,0.));
/// ENDSEC;
/// END-ISO-10303-21;
/// ";
/// let exchange = part21::parse(text).unwrap();
/// assert_eq!(exchange.header.schemas(), ["AUTOMOTIVE_DESIGN"]);
/// let point = exchange.instances[&1].record("CARTESIAN_POINT").unwrap();
/// assert_eq!(
///     point.params[1],
///     Param::List(vec![Param::Real(1.0), Param::Real(-0.25), Param::Real(0.0)])
/// );
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Exchange {
    /// The header section.
    pub header: Header,
    /// Every instance of every `DATA` section, by its `#id`.
    pub instances: BTreeMap<u64, Instance>,
}

/// The header section: its entities in the order written, the three
/// Part 21 requires among them.
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    /// Every header entity, in order.
    pub records: Vec<Record>,
}

impl Header {
    /// The first header entity named `name`, if any.
    pub fn record(&self, name: &str) -> Option<&Record> {
        self.records.iter().find(|r| r.name == name)
    }

    /// The schema names `FILE_SCHEMA` lists, in order: what the file says
    /// it is an instance of (`AUTOMOTIVE_DESIGN` for AP214,
    /// `CONFIG_CONTROL_DESIGN` for AP203, `AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF`
    /// for AP242). Only the strings in its list; empty when it holds none.
    pub fn schemas(&self) -> Vec<&str> {
        let Some(Param::List(names)) = self.record("FILE_SCHEMA").and_then(|r| r.params.first())
        else {
            return Vec::new();
        };
        names
            .iter()
            .filter_map(|p| match p {
                Param::String(s) => Some(s.as_str()),
                _ => None,
            })
            .collect()
    }
}

/// One entity's name and its parameters: a simple instance, a partial
/// entity of a complex one, or a header entity.
#[derive(Debug, Clone, PartialEq)]
pub struct Record {
    /// The entity's keyword, upper-cased.
    pub name: String,
    /// Its parameters, in order.
    pub params: Vec<Param>,
}

/// An instance of a `DATA` section: `#id = NAME(…)`, or the external
/// mapping `#id = (A(…) B(…) …)` of a complex entity, its partial
/// entities in the order written.
#[derive(Debug, Clone, PartialEq)]
pub enum Instance {
    /// One entity.
    Simple(Record),
    /// Several partial entities of one complex instance.
    Complex(Vec<Record>),
}

impl Instance {
    /// Its records: the one of a simple instance, every partial entity of
    /// a complex one.
    pub fn records(&self) -> &[Record] {
        match self {
            Instance::Simple(r) => core::slice::from_ref(r),
            Instance::Complex(rs) => rs,
        }
    }

    /// The record named `name`: a simple instance's own, when it has that
    /// name, or a complex one's partial entity of that name.
    pub fn record(&self, name: &str) -> Option<&Record> {
        self.records().iter().find(|r| r.name == name)
    }
}

/// A parameter, as written: nothing is resolved or converted.
#[derive(Debug, Clone, PartialEq)]
pub enum Param {
    /// An integer.
    Integer(i64),
    /// A real, finite.
    Real(f64),
    /// A string, its encodings decoded.
    String(String),
    /// An enumeration value without its dots, upper-cased: `T`, `F`, `U`
    /// for a logical.
    Enumeration(String),
    /// A binary, its bits most significant first, the unused leading bits
    /// dropped.
    Binary(Vec<bool>),
    /// A reference to an entity instance, `#id`.
    Ref(u64),
    /// A typed parameter, `NAME(value)`: a select's type made explicit.
    Typed(String, Box<Param>),
    /// An aggregate.
    List(Vec<Param>),
    /// `$`: no value.
    Unset,
    /// `*`: a value derived by the schema.
    Derived,
}

/// Why a file is not Part 21, and where.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[error("line {line}, column {column}{}: {kind}", instance.map(|id| format!(", in #{id}")).unwrap_or_default())]
pub struct Part21Error {
    /// The line, from 1.
    pub line: u32,
    /// The column, from 1, in characters.
    pub column: u32,
    /// The instance being read, if the error is inside one.
    pub instance: Option<u64>,
    /// What went wrong.
    pub kind: Part21ErrorKind,
}

/// What [`Part21Error`] found.
#[derive(Debug, Clone, PartialEq)]
pub enum Part21ErrorKind {
    /// A token where the grammar has no place for it, or a token that is
    /// not one: the reason says which.
    Malformed(String),
    /// A string with no closing apostrophe.
    UnterminatedString,
    /// A comment with no closing `*/`.
    UnterminatedComment,
    /// The text ends before `END-ISO-10303-21;`.
    UnexpectedEnd,
    /// A second instance with an id already taken.
    DuplicateId(u64),
    /// A header entity Part 21 requires is missing.
    MissingHeader(&'static str),
    /// A construct of the grammar this parser does not read: the
    /// edition-3 sections and value instances.
    Unsupported(&'static str),
}

impl fmt::Display for Part21ErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Part21ErrorKind::Malformed(reason) => write!(f, "{reason}"),
            Part21ErrorKind::UnterminatedString => write!(f, "a string is not closed"),
            Part21ErrorKind::UnterminatedComment => write!(f, "a comment is not closed"),
            Part21ErrorKind::UnexpectedEnd => write!(f, "the file ends before END-ISO-10303-21"),
            Part21ErrorKind::DuplicateId(id) => write!(f, "#{id} is defined twice"),
            Part21ErrorKind::MissingHeader(name) => write!(f, "the header has no {name}"),
            Part21ErrorKind::Unsupported(what) => write!(f, "{what} is not read"),
        }
    }
}

/// The exchange structure in `text`. Errors: the first place `text`
/// leaves the grammar, as a [`Part21Error`] naming its line, column and
/// instance.
///
/// ```
/// use arris_io::step::part21::{self, Part21ErrorKind};
///
/// let text = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n#1=DIRECTION('',(1.,0.,0.));\n#1=DIRECTION('',(0.,1.,0.));\nENDSEC;\nEND-ISO-10303-21;\n";
/// let e = part21::parse(text).unwrap_err();
/// assert_eq!((e.line, e.instance, e.kind), (9, Some(1), Part21ErrorKind::DuplicateId(1)));
/// ```
pub fn parse(text: &str) -> Result<Exchange, Part21Error> {
    Parser::new(text).exchange()
}

/// A token and where it starts.
#[derive(Debug, Clone, PartialEq)]
enum Token {
    /// A keyword, upper-cased: an entity name, a section, `ISO-10303-21`.
    Keyword(String),
    /// `#id`.
    Entity(u64),
    Integer(i64),
    Real(f64),
    String(String),
    Enumeration(String),
    Binary(Vec<bool>),
    Open,
    Close,
    Comma,
    Semicolon,
    Equals,
    Dollar,
    Star,
    End,
}

impl Token {
    fn describe(&self) -> String {
        match self {
            Token::Keyword(k) => format!("`{k}`"),
            Token::Entity(id) => format!("`#{id}`"),
            Token::Integer(i) => format!("the integer {i}"),
            Token::Real(r) => format!("the real {r}"),
            Token::String(_) => "a string".into(),
            Token::Enumeration(e) => format!("`.{e}.`"),
            Token::Binary(_) => "a binary".into(),
            Token::Open => "`(`".into(),
            Token::Close => "`)`".into(),
            Token::Comma => "`,`".into(),
            Token::Semicolon => "`;`".into(),
            Token::Equals => "`=`".into(),
            Token::Dollar => "`$`".into(),
            Token::Star => "`*`".into(),
            Token::End => "the end of the file".into(),
        }
    }
}

/// How deep lists and typed parameters may nest. The B-Rep subset nests
/// three deep (a B-spline surface's control net in a list); a file deeper
/// than this is refused rather than allowed to exhaust the stack. A
/// structural bound, not a tolerance.
const MAX_NESTING: usize = 64;

struct Parser<'a> {
    text: &'a str,
    bytes: &'a [u8],
    pos: usize,
    line: u32,
    /// Where the current line starts, in bytes.
    line_start: usize,
    /// The bytes of the current line so far that continue a character:
    /// a column counts characters, and counting them afresh at every
    /// token would be quadratic in a file written on one line.
    continuation: usize,
    /// The instance being read.
    instance: Option<u64>,
    /// The token looked at and where it starts: `(pos, line, column)`.
    peeked: Option<(Token, u32, u32)>,
}

impl<'a> Parser<'a> {
    fn new(text: &'a str) -> Self {
        Parser {
            text,
            bytes: text.as_bytes(),
            pos: 0,
            line: 1,
            line_start: 0,
            continuation: 0,
            instance: None,
            peeked: None,
        }
    }

    /// The column of the current position, in characters from 1.
    fn column(&self) -> u32 {
        u32::try_from(self.pos - self.line_start - self.continuation + 1).unwrap_or(u32::MAX)
    }

    fn error_at(&self, line: u32, column: u32, kind: Part21ErrorKind) -> Part21Error {
        Part21Error {
            line,
            column,
            instance: self.instance,
            kind,
        }
    }

    fn error_here(&self, kind: Part21ErrorKind) -> Part21Error {
        self.error_at(self.line, self.column(), kind)
    }

    fn newline(&mut self) {
        self.line = self.line.saturating_add(1);
        self.line_start = self.pos;
        self.continuation = 0;
    }

    /// Past whitespace and comments.
    fn skip_space(&mut self) -> Result<(), Part21Error> {
        while self.pos < self.bytes.len() {
            match self.bytes[self.pos] {
                b'\n' => {
                    self.pos += 1;
                    self.newline();
                }
                b' ' | b'\t' | b'\r' | 0x0b | 0x0c => self.pos += 1,
                b'/' if self.bytes.get(self.pos + 1) == Some(&b'*') => {
                    let (line, column) = (self.line, self.column());
                    self.pos += 2;
                    loop {
                        match self.bytes.get(self.pos) {
                            None => {
                                return Err(self.error_at(
                                    line,
                                    column,
                                    Part21ErrorKind::UnterminatedComment,
                                ));
                            }
                            Some(b'*') if self.bytes.get(self.pos + 1) == Some(&b'/') => {
                                self.pos += 2;
                                break;
                            }
                            Some(b'\n') => {
                                self.pos += 1;
                                self.newline();
                            }
                            Some(&b) => {
                                if b & 0xc0 == 0x80 {
                                    self.continuation += 1;
                                }
                                self.pos += 1;
                            }
                        }
                    }
                }
                _ => break,
            }
        }
        Ok(())
    }

    fn peek(&mut self) -> Result<&Token, Part21Error> {
        if self.peeked.is_none() {
            let t = self.lex()?;
            self.peeked = Some(t);
        }
        match &self.peeked {
            Some((t, _, _)) => Ok(t),
            None => Err(self.error_here(Part21ErrorKind::UnexpectedEnd)),
        }
    }

    /// The next token and where it starts.
    fn next(&mut self) -> Result<(Token, u32, u32), Part21Error> {
        match self.peeked.take() {
            Some(t) => Ok(t),
            None => self.lex(),
        }
    }

    fn malformed(&self, line: u32, column: u32, reason: String) -> Part21Error {
        self.error_at(line, column, Part21ErrorKind::Malformed(reason))
    }

    /// Reads one token.
    fn lex(&mut self) -> Result<(Token, u32, u32), Part21Error> {
        self.skip_space()?;
        let (line, column) = (self.line, self.column());
        let Some(&c) = self.bytes.get(self.pos) else {
            return Ok((Token::End, line, column));
        };
        let token = match c {
            b'(' => self.single(Token::Open),
            b')' => self.single(Token::Close),
            b',' => self.single(Token::Comma),
            b';' => self.single(Token::Semicolon),
            b'=' => self.single(Token::Equals),
            b'$' => self.single(Token::Dollar),
            b'*' => self.single(Token::Star),
            b'#' => {
                self.pos += 1;
                let digits = self.take_while(|b| b.is_ascii_digit());
                let id = digits.parse::<u64>().map_err(|_| {
                    self.malformed(
                        line,
                        column,
                        "`#` is not followed by an instance number".into(),
                    )
                })?;
                Token::Entity(id)
            }
            b'@' => {
                return Err(self.error_at(
                    line,
                    column,
                    Part21ErrorKind::Unsupported("a value instance (`@`, edition 3)"),
                ));
            }
            b'\'' => Token::String(self.string(line, column)?),
            b'"' => Token::Binary(self.binary(line, column)?),
            b'.' => {
                self.pos += 1;
                let name = self.take_while(|b| b.is_ascii_alphanumeric() || b == b'_');
                if name.is_empty() || self.bytes.get(self.pos) != Some(&b'.') {
                    return Err(self.malformed(
                        line,
                        column,
                        "an enumeration is not `.NAME.`".into(),
                    ));
                }
                self.pos += 1;
                Token::Enumeration(name.to_ascii_uppercase())
            }
            b'+' | b'-' | b'0'..=b'9' => self.number(line, column)?,
            b'A'..=b'Z' | b'a'..=b'z' | b'_' | b'!' => {
                let start = self.pos;
                self.pos += 1;
                self.take_while(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-');
                Token::Keyword(self.text[start..self.pos].to_ascii_uppercase())
            }
            _ => {
                let shown = self
                    .text
                    .get(self.pos..)
                    .and_then(|s| s.chars().next())
                    .unwrap_or('?');
                return Err(self.malformed(line, column, format!("`{shown}` starts no token")));
            }
        };
        Ok((token, line, column))
    }

    fn single(&mut self, t: Token) -> Token {
        self.pos += 1;
        t
    }

    /// The ASCII run from here on while `keep` holds.
    fn take_while(&mut self, keep: impl Fn(u8) -> bool) -> &'a str {
        let start = self.pos;
        while self.bytes.get(self.pos).is_some_and(|&b| keep(b)) {
            self.pos += 1;
        }
        // Only ASCII bytes were taken, so the slice is on char boundaries.
        &self.text[start..self.pos]
    }

    /// An integer or a real: `[sign] digits [. digits [E [sign] digits]]`.
    fn number(&mut self, line: u32, column: u32) -> Result<Token, Part21Error> {
        let start = self.pos;
        if matches!(self.bytes[self.pos], b'+' | b'-') {
            self.pos += 1;
        }
        if self.take_while(|b| b.is_ascii_digit()).is_empty() {
            return Err(self.malformed(line, column, "a sign is not followed by a digit".into()));
        }
        if self.bytes.get(self.pos) != Some(&b'.') {
            let text = &self.text[start..self.pos];
            return text.parse::<i64>().map(Token::Integer).map_err(|_| {
                self.malformed(
                    line,
                    column,
                    format!("the integer {text} does not fit 64 bits"),
                )
            });
        }
        self.pos += 1;
        self.take_while(|b| b.is_ascii_digit());
        if matches!(self.bytes.get(self.pos), Some(b'E' | b'e')) {
            self.pos += 1;
            if matches!(self.bytes.get(self.pos), Some(b'+' | b'-')) {
                self.pos += 1;
            }
            if self.take_while(|b| b.is_ascii_digit()).is_empty() {
                return Err(self.malformed(line, column, "an exponent has no digits".into()));
            }
        }
        let text = &self.text[start..self.pos];
        match text.parse::<f64>() {
            Ok(x) if x.is_finite() => Ok(Token::Real(x)),
            _ => Err(self.malformed(
                line,
                column,
                format!("the real {text} is not a finite number"),
            )),
        }
    }

    /// A string from its opening apostrophe: `''` is an apostrophe, `\\` a
    /// backslash, and the control directives decode.
    fn string(&mut self, line: u32, column: u32) -> Result<String, Part21Error> {
        self.pos += 1;
        let mut out = String::new();
        let bad = |p: &Self, what: &str| p.malformed(line, column, format!("a string has {what}"));
        loop {
            let Some(&b) = self.bytes.get(self.pos) else {
                return Err(self.error_at(line, column, Part21ErrorKind::UnterminatedString));
            };
            match b {
                b'\'' => {
                    if self.bytes.get(self.pos + 1) == Some(&b'\'') {
                        out.push('\'');
                        self.pos += 2;
                    } else {
                        self.pos += 1;
                        return Ok(out);
                    }
                }
                b'\\' => {
                    let rest = &self.bytes[self.pos..];
                    if rest.starts_with(b"\\\\") {
                        out.push('\\');
                        self.pos += 2;
                    } else if rest.starts_with(b"\\S\\") {
                        // A character of the upper half of the current
                        // page, ISO 8859-1 unless `\P?\` chose another
                        // (whose upper half is not mapped: Latin-1 is
                        // what every writer of the subset uses).
                        let c = rest.get(3).copied().filter(|c| (0x20..0x7f).contains(c));
                        let c = c.ok_or_else(|| bad(self, "`\\S\\` without a character"))?;
                        out.push(char::from(c + 0x80));
                        self.pos += 4;
                    } else if rest.len() >= 4
                        && rest[1] == b'P'
                        && rest[2].is_ascii_uppercase()
                        && rest[3] == b'\\'
                    {
                        // A page directive, `\PA\` to `\PI\`: the page is
                        // not kept. Its letter is checked, so a line break
                        // is never skipped as one.
                        self.pos += 4;
                    } else if rest.starts_with(b"\\X2\\") || rest.starts_with(b"\\X4\\") {
                        let width = if rest[2] == b'2' { 4 } else { 8 };
                        self.pos += 4;
                        loop {
                            let tail = &self.bytes[self.pos..];
                            if tail.starts_with(b"\\X0\\") {
                                self.pos += 4;
                                break;
                            }
                            let digits = tail
                                .get(..width)
                                .and_then(|d| core::str::from_utf8(d).ok())
                                .and_then(|d| u32::from_str_radix(d, 16).ok())
                                .ok_or_else(|| bad(self, "an `\\X2\\` or `\\X4\\` run that is not hex closed by `\\X0\\`"))?;
                            let c = char::from_u32(digits)
                                .ok_or_else(|| bad(self, "a code point that is not a character"))?;
                            out.push(c);
                            self.pos += width;
                        }
                    } else if rest.starts_with(b"\\X\\") {
                        let c = rest
                            .get(3..5)
                            .and_then(|d| core::str::from_utf8(d).ok())
                            .and_then(|d| u8::from_str_radix(d, 16).ok())
                            .ok_or_else(|| bad(self, "`\\X\\` without two hex digits"))?;
                        out.push(char::from(c));
                        self.pos += 5;
                    } else if rest.starts_with(b"\\N\\") {
                        out.push('\n');
                        self.pos += 3;
                    } else if rest.starts_with(b"\\T\\") {
                        out.push('\t');
                        self.pos += 3;
                    } else {
                        return Err(bad(self, "a `\\` that starts no directive"));
                    }
                }
                b'\n' => {
                    // A line break inside a string is not part of it: the
                    // writer wrapped a long line.
                    self.pos += 1;
                    self.newline();
                }
                b'\r' => self.pos += 1,
                _ => {
                    // Anything else as written, UTF-8 included, which the
                    // grammar does not allow but writers put there.
                    let ch = self.text[self.pos..].chars().next().unwrap_or('\u{fffd}');
                    out.push(ch);
                    self.pos += ch.len_utf8();
                    self.continuation += ch.len_utf8() - 1;
                }
            }
        }
    }

    /// A binary from its opening quote: a digit `0`–`3` of unused leading
    /// bits, then hex, then the closing quote.
    fn binary(&mut self, line: u32, column: u32) -> Result<Vec<bool>, Part21Error> {
        self.pos += 1;
        let digits = self.take_while(|b| b.is_ascii_hexdigit());
        if self.bytes.get(self.pos) != Some(&b'"') {
            return Err(self.malformed(line, column, "a binary is not hex closed by `\"`".into()));
        }
        self.pos += 1;
        let mut chars = digits.chars();
        let unused = chars
            .next()
            .and_then(|c| c.to_digit(10))
            .filter(|&u| u <= 3 && (u == 0 || digits.len() > 1))
            .ok_or_else(|| {
                self.malformed(
                    line,
                    column,
                    "a binary does not start with its unused bits, 0 to 3".into(),
                )
            })?;
        let mut bits = Vec::with_capacity(4 * (digits.len() - 1));
        for c in chars {
            // Every char is hex: `take_while` above kept only those.
            let nibble = c.to_digit(16).unwrap_or(0);
            bits.extend((0..4).rev().map(|k| nibble >> k & 1 == 1));
        }
        bits.drain(..unused as usize);
        Ok(bits)
    }

    fn expect(&mut self, want: Token) -> Result<(), Part21Error> {
        let (t, line, column) = self.next()?;
        if t == want {
            return Ok(());
        }
        Err(self.unexpected(t, line, column, &want.describe()))
    }

    fn unexpected(&self, t: Token, line: u32, column: u32, wanted: &str) -> Part21Error {
        if t == Token::End {
            return self.error_at(line, column, Part21ErrorKind::UnexpectedEnd);
        }
        self.malformed(
            line,
            column,
            format!("{} where {wanted} belongs", t.describe()),
        )
    }

    fn expect_keyword(&mut self, name: &str) -> Result<(), Part21Error> {
        let (t, line, column) = self.next()?;
        match t {
            Token::Keyword(k) if k == name => Ok(()),
            t => Err(self.unexpected(t, line, column, &format!("`{name}`"))),
        }
    }

    fn exchange(mut self) -> Result<Exchange, Part21Error> {
        self.expect_keyword("ISO-10303-21")?;
        self.expect(Token::Semicolon)?;
        self.expect_keyword("HEADER")?;
        self.expect(Token::Semicolon)?;
        let mut records = Vec::new();
        loop {
            match self.peek()? {
                Token::Keyword(k) if k == "ENDSEC" => break,
                _ => {
                    records.push(self.record()?);
                    self.expect(Token::Semicolon)?;
                }
            }
        }
        // A missing entity is noticed at the section's end.
        let (_, header_line, header_column) = self.next()?;
        self.expect(Token::Semicolon)?;
        for name in ["FILE_DESCRIPTION", "FILE_NAME", "FILE_SCHEMA"] {
            if !records.iter().any(|r| r.name == name) {
                return Err(self.error_at(
                    header_line,
                    header_column,
                    Part21ErrorKind::MissingHeader(name),
                ));
            }
        }
        let mut instances = BTreeMap::new();
        let mut sections = 0usize;
        loop {
            let (t, line, column) = self.next()?;
            let Token::Keyword(k) = t else {
                return Err(self.unexpected(t, line, column, "a section"));
            };
            match k.as_str() {
                "DATA" => {
                    if self.peek()? == &Token::Open {
                        // A named section's parameters: its name and
                        // schema, which one file's sections share here.
                        self.next()?;
                        self.params(0)?;
                    }
                    self.expect(Token::Semicolon)?;
                    self.data(&mut instances)?;
                    sections += 1;
                }
                "END-ISO-10303-21" => {
                    self.expect(Token::Semicolon)?;
                    if sections == 0 {
                        return Err(self.malformed(
                            line,
                            column,
                            "the file has no DATA section".into(),
                        ));
                    }
                    return Ok(Exchange {
                        header: Header { records },
                        instances,
                    });
                }
                "ANCHOR" => {
                    return Err(self.error_at(
                        line,
                        column,
                        Part21ErrorKind::Unsupported("an ANCHOR section (edition 3)"),
                    ));
                }
                "REFERENCE" => {
                    return Err(self.error_at(
                        line,
                        column,
                        Part21ErrorKind::Unsupported("a REFERENCE section (edition 3)"),
                    ));
                }
                "SIGNATURE" => {
                    return Err(self.error_at(
                        line,
                        column,
                        Part21ErrorKind::Unsupported("a SIGNATURE section (edition 3)"),
                    ));
                }
                other => {
                    return Err(self.malformed(
                        line,
                        column,
                        format!("`{other}` is not a section"),
                    ));
                }
            }
        }
    }

    /// The instances of one `DATA` section, up to its `ENDSEC;`.
    fn data(&mut self, instances: &mut BTreeMap<u64, Instance>) -> Result<(), Part21Error> {
        loop {
            let (t, line, column) = self.next()?;
            let id = match t {
                Token::Entity(id) => id,
                Token::Keyword(k) if k == "ENDSEC" => {
                    return self.expect(Token::Semicolon);
                }
                t => return Err(self.unexpected(t, line, column, "an instance or `ENDSEC`")),
            };
            self.instance = Some(id);
            self.expect(Token::Equals)?;
            let instance = if self.peek()? == &Token::Open {
                self.next()?;
                let mut parts = Vec::new();
                while self.peek()? != &Token::Close {
                    parts.push(self.record()?);
                }
                self.next()?;
                if parts.is_empty() {
                    return Err(self.malformed(
                        line,
                        column,
                        "a complex instance has no partial entity".into(),
                    ));
                }
                Instance::Complex(parts)
            } else {
                Instance::Simple(self.record()?)
            };
            self.expect(Token::Semicolon)?;
            if instances.insert(id, instance).is_some() {
                return Err(self.error_at(line, column, Part21ErrorKind::DuplicateId(id)));
            }
            self.instance = None;
        }
    }

    /// `NAME(params)`.
    fn record(&mut self) -> Result<Record, Part21Error> {
        let (t, line, column) = self.next()?;
        let Token::Keyword(name) = t else {
            return Err(self.unexpected(t, line, column, "an entity name"));
        };
        self.expect(Token::Open)?;
        let params = self.params(0)?;
        Ok(Record { name, params })
    }

    /// Parameters after an opening parenthesis, up to and past its close.
    fn params(&mut self, depth: usize) -> Result<Vec<Param>, Part21Error> {
        let mut out = Vec::new();
        if self.peek()? == &Token::Close {
            self.next()?;
            return Ok(out);
        }
        loop {
            out.push(self.param(depth)?);
            let (t, line, column) = self.next()?;
            match t {
                Token::Comma => {}
                Token::Close => return Ok(out),
                t => return Err(self.unexpected(t, line, column, "`,` or `)`")),
            }
        }
    }

    fn param(&mut self, depth: usize) -> Result<Param, Part21Error> {
        let (t, line, column) = self.next()?;
        if depth >= MAX_NESTING {
            return Err(self.malformed(
                line,
                column,
                format!("parameters nest deeper than {MAX_NESTING}"),
            ));
        }
        Ok(match t {
            Token::Integer(i) => Param::Integer(i),
            Token::Real(x) => Param::Real(x),
            Token::String(s) => Param::String(s),
            Token::Enumeration(e) => Param::Enumeration(e),
            Token::Binary(b) => Param::Binary(b),
            Token::Entity(id) => Param::Ref(id),
            Token::Dollar => Param::Unset,
            Token::Star => Param::Derived,
            Token::Open => Param::List(self.params(depth + 1)?),
            Token::Keyword(name) => {
                self.expect(Token::Open)?;
                let value = self.param(depth + 1)?;
                self.expect(Token::Close)?;
                Param::Typed(name, Box::new(value))
            }
            t => return Err(self.unexpected(t, line, column, "a parameter")),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEAD: &str = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('x'),'2;1');\nFILE_NAME('a','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\n";

    /// A file of `data` as its one `DATA` section's body.
    fn file(data: &str) -> String {
        format!("{HEAD}DATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n")
    }

    /// The parameters of `#1`, a simple instance of `data`.
    fn params_of(data: &str) -> Vec<Param> {
        let x = parse(&file(data)).unwrap();
        match &x.instances[&1] {
            Instance::Simple(r) => r.params.clone(),
            Instance::Complex(_) => panic!("complex"),
        }
    }

    fn error_of(text: &str) -> Part21Error {
        parse(text).unwrap_err()
    }

    #[test]
    fn the_header_keeps_its_entities_and_names_its_schemas() {
        let x = parse(&file("#1=A();")).unwrap();
        let names: Vec<_> = x.header.records.iter().map(|r| r.name.as_str()).collect();
        assert_eq!(names, ["FILE_DESCRIPTION", "FILE_NAME", "FILE_SCHEMA"]);
        assert_eq!(x.header.schemas(), ["AUTOMOTIVE_DESIGN"]);
        assert_eq!(
            x.header.record("FILE_DESCRIPTION").unwrap().params[1],
            Param::String("2;1".into())
        );
    }

    #[test]
    fn a_missing_header_entity_is_named() {
        let text = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('','',(''),(''),'','','');\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n";
        let e = error_of(text);
        assert_eq!(e.kind, Part21ErrorKind::MissingHeader("FILE_SCHEMA"));
        assert_eq!((e.line, e.column), (5, 1));
    }

    #[test]
    fn several_data_sections_one_named_are_one_map() {
        let text = format!(
            "{HEAD}DATA;\n#1=A();\nENDSEC;\nDATA(('second'),('AUTOMOTIVE_DESIGN'));\n#2=B(#1);\nENDSEC;\nEND-ISO-10303-21;\n"
        );
        let x = parse(&text).unwrap();
        assert_eq!(x.instances.keys().copied().collect::<Vec<_>>(), [1, 2]);
        assert_eq!(x.instances[&2].record("B").unwrap().params, [Param::Ref(1)]);
        // An id taken in the first section is taken in the second.
        let text =
            format!("{HEAD}DATA;\n#1=A();\nENDSEC;\nDATA;\n#1=B();\nENDSEC;\nEND-ISO-10303-21;\n");
        let e = error_of(&text);
        assert_eq!((e.line, e.kind), (11, Part21ErrorKind::DuplicateId(1)));
    }

    #[test]
    fn a_complex_instance_keeps_its_partial_entities_in_order() {
        let x = parse(&file(
            "#1=(BOUNDED_CURVE()B_SPLINE_CURVE(2,(#2,#3,#4),.UNSPECIFIED.,.F.,.F.)\nRATIONAL_B_SPLINE_CURVE((1.,0.7,1.)));",
        ))
        .unwrap();
        let i = &x.instances[&1];
        let names: Vec<_> = i.records().iter().map(|r| r.name.as_str()).collect();
        assert_eq!(
            names,
            ["BOUNDED_CURVE", "B_SPLINE_CURVE", "RATIONAL_B_SPLINE_CURVE"]
        );
        assert_eq!(
            i.record("RATIONAL_B_SPLINE_CURVE").unwrap().params,
            [Param::List(vec![
                Param::Real(1.0),
                Param::Real(0.7),
                Param::Real(1.0)
            ])]
        );
        assert_eq!(
            i.record("B_SPLINE_CURVE").unwrap().params[2],
            Param::Enumeration("UNSPECIFIED".into())
        );
        let e = error_of(&file("#1=();"));
        assert!(matches!(e.kind, Part21ErrorKind::Malformed(_)), "{e}");
    }

    #[test]
    fn integers_and_reals_in_every_spelling() {
        let p = params_of("#1=A(0,-7,+12,1.,-1.E-5,2.5E+3,+0.E0,0.25,3.e2,1.0E308);");
        let reals: Vec<f64> = p[3..]
            .iter()
            .map(|x| match x {
                Param::Real(r) => *r,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(
            p[..3],
            [Param::Integer(0), Param::Integer(-7), Param::Integer(12)]
        );
        assert_eq!(reals, [1.0, -1e-5, 2500.0, 0.0, 0.25, 300.0, 1e308]);
    }

    #[test]
    fn a_number_out_of_range_or_cut_short_is_malformed() {
        for (text, line) in [
            ("#1=A(1.E999);", 8),
            ("#1=A(99999999999999999999);", 8),
            ("#1=A(1.5E);", 8),
            ("#1=A(-);", 8),
            ("#1=A(\n.5);", 9),
        ] {
            let e = error_of(&file(text));
            assert!(
                matches!(e.kind, Part21ErrorKind::Malformed(_)),
                "{text}: {e}"
            );
            assert_eq!((e.line, e.instance), (line, Some(1)), "{text}: {e}");
        }
    }

    #[test]
    fn strings_decode_quotes_and_every_encoding() {
        let p = params_of(
            r"#1=A('it''s','a\\b','\S\a','\X\E9','\X2\03B103B2\X0\','\X4\0001F600\X0\','\PA\\S\D','');",
        );
        let strings: Vec<&str> = p
            .iter()
            .map(|x| match x {
                Param::String(s) => s.as_str(),
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(strings, ["it's", "a\\b", "á", "é", "αβ", "😀", "Ä", ""]);
    }

    #[test]
    fn a_bad_directive_or_an_open_string_names_where_it_starts() {
        let e = error_of(&file("#1=A('abc\\X2\\00E);"));
        assert!(matches!(e.kind, Part21ErrorKind::Malformed(_)), "{e}");
        assert_eq!((e.line, e.column, e.instance), (8, 6, Some(1)));
        // A page directive's letter is a letter, never a line break the
        // line count would miss (the fuzz target's first finding).
        let e = error_of(&file("#1=A('\\P\n\\',x);"));
        assert!(matches!(e.kind, Part21ErrorKind::Malformed(_)), "{e}");
        assert_eq!((e.line, e.column, e.instance), (8, 6, Some(1)));
        let e = error_of(&file("#1=A('abc);\n#2=B();"));
        assert_eq!(e.kind, Part21ErrorKind::UnterminatedString);
        assert_eq!((e.line, e.column, e.instance), (8, 6, Some(1)));
    }

    #[test]
    fn enumerations_binaries_and_the_empty_values() {
        let p = params_of(r#"#1=A(.T.,.unspecified.,"0F","2A",$,*);"#);
        assert_eq!(p[0], Param::Enumeration("T".into()));
        assert_eq!(p[1], Param::Enumeration("UNSPECIFIED".into()));
        assert_eq!(p[2], Param::Binary(vec![true; 4]));
        // `A` is 1010 with its two leading bits unused.
        assert_eq!(p[3], Param::Binary(vec![true, false]));
        assert_eq!(p[4..], [Param::Unset, Param::Derived]);
        for bad in [r#"#1=A("4F");"#, r#"#1=A("0G");"#, "#1=A(.T);"] {
            let e = error_of(&file(bad));
            assert!(
                matches!(e.kind, Part21ErrorKind::Malformed(_)),
                "{bad}: {e}"
            );
        }
    }

    #[test]
    fn typed_parameters_references_and_nested_lists() {
        let p = params_of(
            "#1=A(LENGTH_MEASURE(2.5),#12,((1,2),(3,())),PARAMETER_VALUE(POSITIVE_LENGTH_MEASURE(1.)));",
        );
        assert_eq!(
            p[0],
            Param::Typed("LENGTH_MEASURE".into(), Box::new(Param::Real(2.5)))
        );
        assert_eq!(p[1], Param::Ref(12));
        assert_eq!(
            p[2],
            Param::List(vec![
                Param::List(vec![Param::Integer(1), Param::Integer(2)]),
                Param::List(vec![Param::Integer(3), Param::List(vec![])]),
            ])
        );
        assert!(
            matches!(&p[3], Param::Typed(n, inner) if n == "PARAMETER_VALUE" && matches!(**inner, Param::Typed(..)))
        );
        // Nesting past the bound is refused, not recursed into.
        let deep = format!("#1=A({}1{});", "(".repeat(100), ")".repeat(100));
        assert!(matches!(
            error_of(&file(&deep)).kind,
            Part21ErrorKind::Malformed(_)
        ));
    }

    #[test]
    fn comments_go_anywhere_a_space_does() {
        let x = parse(&file(
            "/* a\ncomment */#1/**/=/**/A(1/* in */,\n/* x */2)/**/;",
        ))
        .unwrap();
        assert_eq!(
            x.instances[&1].record("A").unwrap().params,
            [Param::Integer(1), Param::Integer(2)]
        );
        let e = error_of(&file("#1=A(); /* never closed"));
        assert_eq!(e.kind, Part21ErrorKind::UnterminatedComment);
        assert_eq!((e.line, e.column, e.instance), (8, 9, None));
    }

    #[test]
    fn a_malformed_token_names_its_line_column_and_instance() {
        let e = error_of(&file("#1=A();\n#2=B(1 2);"));
        assert!(
            matches!(&e.kind, Part21ErrorKind::Malformed(r) if r.contains("`,` or `)`")),
            "{e}"
        );
        assert_eq!((e.line, e.column, e.instance), (9, 8, Some(2)));
        let e = error_of(&file("#1=A(%);"));
        assert_eq!((e.line, e.column), (8, 6));
        let e = error_of(&file("#1=A()"));
        assert!(matches!(e.kind, Part21ErrorKind::Malformed(_)), "{e}");
        // A column counts characters, not bytes.
        let e = error_of(&file("#1=A('é',%);"));
        assert_eq!((e.line, e.column), (8, 10));
    }

    #[test]
    fn the_edition_3_sections_and_value_instances_are_refused_by_name() {
        let text = format!("{HEAD}ANCHOR;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n");
        let e = error_of(&text);
        assert!(matches!(e.kind, Part21ErrorKind::Unsupported(w) if w.contains("ANCHOR")));
        assert_eq!(e.line, 7);
        let e = error_of(&file("#1=A(@3);"));
        assert!(matches!(e.kind, Part21ErrorKind::Unsupported(w) if w.contains("value instance")));
    }

    #[test]
    fn a_file_on_one_line_parses_in_linear_time() {
        // Two hundred thousand instances and a character past ASCII on
        // one line: a column counted afresh per token would take minutes.
        let data: String = (1..=200_000)
            .map(|i| format!("#{i}=A('é',{i}.5,#{});", i + 1))
            .collect();
        let x = parse(&file(&data)).unwrap();
        assert_eq!(x.instances.len(), 200_000);
        let e = error_of(&file(&format!("{data}%")));
        assert_eq!(e.line, 8);
        assert_eq!(e.column as usize, data.chars().count() + 1);
    }

    #[test]
    fn a_cut_file_is_an_unexpected_end() {
        let whole = file("#1=A(1,'x',(2.));");
        for cut in 0..whole.len() - "END-ISO-10303-21;\n".len() {
            let Some(part) = whole.get(..cut) else {
                continue;
            };
            assert!(parse(part).is_err(), "a file cut at byte {cut} parsed");
        }
        let e = error_of(&whole[..whole.len() - "END-ISO-10303-21;\n".len()]);
        assert_eq!(e.kind, Part21ErrorKind::UnexpectedEnd);
        assert!(matches!(error_of("").kind, Part21ErrorKind::UnexpectedEnd));
        let no_data = format!("{HEAD}END-ISO-10303-21;\n");
        assert!(matches!(
            error_of(&no_data).kind,
            Part21ErrorKind::Malformed(_)
        ));
    }
}
