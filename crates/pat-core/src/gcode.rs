//! G-code tokenizer.
//!
//! Handles the common RS-274 subset emitted by CAM posts: letter/number words,
//! `( )` and `;` comments, `%` tape markers, `/` block delete and line numbers.
//! Parameters (`#1`), expressions (`[..]`) and O-word subroutines are not
//! supported and are reported as errors rather than silently misread.

use serde::Serialize;
use thiserror::Error;

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Word {
    pub letter: char,
    pub value: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Block {
    /// 1-based source line.
    pub line: usize,
    pub words: Vec<Word>,
    pub comments: Vec<String>,
    pub block_delete: bool,
}

impl Block {
    /// Value of the single word with this letter, if present.
    pub fn get(&self, letter: char) -> Option<f64> {
        self.words
            .iter()
            .find(|w| w.letter == letter)
            .map(|w| w.value)
    }

    pub fn has(&self, letter: char) -> bool {
        self.words.iter().any(|w| w.letter == letter)
    }

    /// All values for a letter (e.g. several G codes on one line).
    pub fn all(&self, letter: char) -> impl Iterator<Item = f64> + '_ {
        self.words
            .iter()
            .filter(move |w| w.letter == letter)
            .map(|w| w.value)
    }
}

#[derive(Debug, Error, PartialEq)]
#[error("line {line}: {msg}")]
pub struct ParseError {
    pub line: usize,
    pub msg: String,
}

pub fn parse_program(src: &str) -> Result<Vec<Block>, ParseError> {
    let mut blocks = Vec::new();
    for (n, raw) in src.lines().enumerate() {
        if let Some(b) = parse_line(raw, n + 1)? {
            blocks.push(b);
        }
    }
    Ok(blocks)
}

fn parse_line(raw: &str, line: usize) -> Result<Option<Block>, ParseError> {
    let err = |msg: String| ParseError { line, msg };
    let mut comments = Vec::new();
    let mut code = String::new();
    let mut chars = raw.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '(' => {
                let mut text = String::new();
                loop {
                    match chars.next() {
                        Some(')') => break,
                        Some(ch) => text.push(ch),
                        None => return Err(err("unclosed '(' comment".into())),
                    }
                }
                comments.push(text.trim().to_string());
            }
            ';' => {
                comments.push(chars.by_ref().collect::<String>().trim().to_string());
            }
            c if c.is_whitespace() => {}
            c => code.push(c),
        }
    }

    let mut block_delete = false;
    let mut s = code.as_str();
    if s == "%" || (s.is_empty() && comments.is_empty()) {
        return Ok(None);
    }
    if let Some(rest) = s.strip_prefix('/') {
        block_delete = true;
        s = rest;
    }

    let mut words = Vec::new();
    let bytes: Vec<char> = s.chars().collect();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i];
        if c == '#' || c == '[' {
            return Err(err("parameters and expressions are not supported".into()));
        }
        if !c.is_ascii_alphabetic() {
            return Err(err(format!("unexpected character '{c}'")));
        }
        let letter = c.to_ascii_uppercase();
        i += 1;
        let start = i;
        while i < bytes.len() && (bytes[i].is_ascii_digit() || matches!(bytes[i], '.' | '-' | '+'))
        {
            i += 1;
        }
        let num: String = bytes[start..i].iter().collect();
        if num.is_empty() {
            return Err(err(format!("word '{letter}' has no value")));
        }
        let value: f64 = num
            .parse()
            .map_err(|_| err(format!("bad number '{num}' after '{letter}'")))?;
        if letter == 'O' && i < bytes.len() {
            return Err(err(
                "O-word subroutines and control flow are not supported".into()
            ));
        }
        words.push(Word { letter, value });
    }
    Ok(Some(Block {
        line,
        words,
        comments,
        block_delete,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_words_and_comments() {
        let p = parse_program("N10 G1 X-1.5 Y.25 (cut) F300 ; trailing\n%\n").unwrap();
        assert_eq!(p.len(), 1);
        let b = &p[0];
        assert_eq!(b.get('X'), Some(-1.5));
        assert_eq!(b.get('Y'), Some(0.25));
        assert_eq!(b.get('F'), Some(300.0));
        assert_eq!(b.comments, vec!["cut", "trailing"]);
    }

    #[test]
    fn decimal_g_codes_and_spaces() {
        let p = parse_program("g43.4 h1 x 10").unwrap();
        assert_eq!(p[0].get('G'), Some(43.4));
        assert_eq!(p[0].get('X'), Some(10.0));
    }

    #[test]
    fn rejects_parameters() {
        assert!(parse_program("G1 X#1").is_err());
    }
}
