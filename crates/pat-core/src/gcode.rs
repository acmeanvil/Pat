//! G-code tokenizer.
//!
//! Handles the common RS-274 subset emitted by CAM posts: letter/number words,
//! `( )` and `;` comments, `%` tape markers, `/` block delete and line numbers.
//! O-words are recognised for subroutines (`o100 sub` / `call` / `endsub`,
//! and Fanuc `O1000` program numbers; see [`crate::subs`]). Parameters
//! (`#1`), expressions (`[..]`) and O-word control flow are not supported and
//! are reported as errors rather than silently misread.

use serde::Serialize;
use thiserror::Error;

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Word {
    pub letter: char,
    pub value: f64,
}

/// An O-word: a LinuxCNC subroutine statement (`o100 sub`, `o<name> call`)
/// or a Fanuc program number (`O1000`, no keyword).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct OWord {
    /// Number without leading zeros, or the name inside `<..>`, lower case.
    pub name: String,
    pub keyword: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Block {
    /// 1-based source line.
    pub line: usize,
    pub words: Vec<Word>,
    pub comments: Vec<String>,
    pub block_delete: bool,
    pub oword: Option<OWord>,
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

    // Optional leading line number before an O-word.
    let mut chars_all: Vec<char> = s.chars().collect();
    let mut i = 0;
    if matches!(chars_all.first(), Some('N' | 'n')) {
        let mut j = 1;
        while j < chars_all.len() && chars_all[j].is_ascii_digit() {
            j += 1;
        }
        if matches!(chars_all.get(j), Some('O' | 'o')) {
            i = j;
        }
    }
    let mut oword = None;
    if matches!(chars_all.get(i), Some('O' | 'o')) {
        i += 1;
        let name = if chars_all.get(i) == Some(&'<') {
            let start = i + 1;
            let end = chars_all[start..]
                .iter()
                .position(|&c| c == '>')
                .map(|p| start + p)
                .ok_or_else(|| err("unclosed '<' in O-word name".into()))?;
            i = end + 1;
            chars_all[start..end]
                .iter()
                .collect::<String>()
                .to_ascii_lowercase()
        } else {
            let start = i;
            while i < chars_all.len() && chars_all[i].is_ascii_digit() {
                i += 1;
            }
            if start == i {
                return Err(err("O-word has no number or name".into()));
            }
            let digits: String = chars_all[start..i].iter().collect();
            let trimmed = digits.trim_start_matches('0');
            if trimmed.is_empty() {
                "0".to_string()
            } else {
                trimmed.to_string()
            }
        };
        let kw_end = (i..chars_all.len())
            .find(|&k| !chars_all[k].is_ascii_alphabetic())
            .unwrap_or(chars_all.len());
        let kw: String = chars_all[i..kw_end]
            .iter()
            .collect::<String>()
            .to_ascii_lowercase();
        const KEYWORDS: [&str; 15] = [
            "sub",
            "endsub",
            "call",
            "return",
            "if",
            "elseif",
            "else",
            "endif",
            "while",
            "endwhile",
            "do",
            "repeat",
            "endrepeat",
            "break",
            "continue",
        ];
        if KEYWORDS.contains(&kw.as_str()) {
            let rest: String = chars_all[kw_end..].iter().collect();
            if !matches!(kw.as_str(), "sub" | "endsub" | "call" | "return") {
                return Err(err(format!("O-word control flow '{kw}' is not supported")));
            }
            if !rest.is_empty() {
                return Err(err(format!(
                    "O-word '{kw}' with arguments or expressions is not supported"
                )));
            }
            return Ok(Some(Block {
                line,
                words: Vec::new(),
                comments,
                block_delete,
                oword: Some(OWord {
                    name,
                    keyword: Some(kw),
                }),
            }));
        }
        // Fanuc program number; the rest of the line is ordinary words.
        oword = Some(OWord {
            name,
            keyword: None,
        });
    }
    let bytes: Vec<char> = std::mem::take(&mut chars_all);
    let mut words = Vec::new();
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
        if letter == 'O' {
            return Err(err("O-word must start the block".into()));
        }
        words.push(Word { letter, value });
    }
    Ok(Some(Block {
        line,
        words,
        comments,
        block_delete,
        oword,
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
    fn parses_owords() {
        let p = parse_program("o100 sub\nO<Drill> call\nN5 o0100 endsub\nO1000\nO2000 G0 X1\n")
            .unwrap();
        let ow: Vec<_> = p.iter().map(|b| b.oword.clone().unwrap()).collect();
        assert_eq!(
            ow[0],
            OWord {
                name: "100".into(),
                keyword: Some("sub".into())
            }
        );
        assert_eq!(ow[1].name, "drill");
        assert_eq!(
            ow[2],
            OWord {
                name: "100".into(),
                keyword: Some("endsub".into())
            }
        );
        assert_eq!(ow[3].keyword, None);
        assert_eq!(p[4].get('X'), Some(1.0));
        assert!(parse_program("o100 if [#1 GT 2]").is_err());
        assert!(parse_program("o100 call [1]").is_err());
    }

    #[test]
    fn rejects_parameters() {
        assert!(parse_program("G1 X#1").is_err());
    }
}
