//! Subroutine expansion.
//!
//! Flattens a parsed program into the sequence of blocks the control would
//! actually execute, so the interpreter and simulator never see calls.
//!
//! Supported forms:
//! - LinuxCNC: `o100 sub` … `o100 endsub`, called with `o100 call`
//!   (no arguments). `o100 return` ends the subroutine early. Definitions are
//!   skipped when reached in normal flow, as LinuxCNC does.
//! - Fanuc / Haas: `M98 P1001` (optional `L` or `K` repeat count, or the
//!   8-digit `P00031001` form = 3 × O1001) calls the block range starting at
//!   `O1001` and ending at `M99`. `M99` in the main program ends it.
//!
//! Every expanded block keeps its original source line, so issues point at
//! the line inside the subroutine.

use crate::gcode::Block;

/// Maximum call nesting, as a guard against recursion.
pub const MAX_DEPTH: usize = 16;

#[derive(Debug, Clone, PartialEq)]
pub struct SubError {
    pub line: usize,
    pub msg: String,
}

pub fn expand(blocks: &[Block]) -> (Vec<Block>, Vec<SubError>) {
    let mut ex = Expander {
        blocks,
        out: Vec::new(),
        errors: Vec::new(),
    };
    ex.run(0, None, 0);
    (ex.out, ex.errors)
}

struct Expander<'a> {
    blocks: &'a [Block],
    out: Vec<Block>,
    errors: Vec<SubError>,
}

enum Flow {
    Continue,
    /// Subroutine or program finished.
    Return,
}

impl Expander<'_> {
    fn err(&mut self, line: usize, msg: String) {
        if !self.errors.iter().any(|e| e.line == line && e.msg == msg) {
            self.errors.push(SubError { line, msg });
        }
    }

    fn kw(b: &Block) -> Option<(&str, &str)> {
        b.oword
            .as_ref()
            .and_then(|o| o.keyword.as_deref().map(|k| (o.name.as_str(), k)))
    }

    /// Index just past the `endsub` matching the `sub` at `start`.
    fn end_of_sub(&self, start: usize, name: &str) -> Option<usize> {
        (start + 1..self.blocks.len())
            .find(|&j| Self::kw(&self.blocks[j]) == Some((name, "endsub")))
            .map(|j| j + 1)
    }

    fn find_linuxcnc_sub(&self, name: &str) -> Option<usize> {
        self.blocks
            .iter()
            .position(|b| Self::kw(b) == Some((name, "sub")))
    }

    fn find_fanuc_program(&self, number: &str) -> Option<usize> {
        self.blocks.iter().position(|b| {
            b.oword
                .as_ref()
                .is_some_and(|o| o.keyword.is_none() && o.name == number)
        })
    }

    /// Execute from `start`. `sub_name` is set when running a LinuxCNC sub
    /// (stop at its endsub); `None` with depth > 0 means a Fanuc sub (stop at M99).
    fn run(&mut self, start: usize, sub_name: Option<&str>, depth: usize) {
        let mut i = start;
        while i < self.blocks.len() {
            let b = &self.blocks[i];
            if let Some((name, kw)) = Self::kw(b) {
                match kw {
                    "sub" => {
                        // Definition reached in normal flow: skip its body.
                        match self.end_of_sub(i, name) {
                            Some(next) => {
                                i = next;
                                continue;
                            }
                            None => {
                                self.err(b.line, format!("o{name} sub has no matching endsub"));
                                return;
                            }
                        }
                    }
                    "endsub" | "return" => {
                        if sub_name.is_some() {
                            return;
                        }
                        self.err(b.line, format!("o{name} {kw} outside a subroutine"));
                    }
                    "call" => {
                        let name = name.to_string();
                        if depth >= MAX_DEPTH {
                            self.err(b.line, format!("call nesting deeper than {MAX_DEPTH}"));
                        } else if let Some(s) = self.find_linuxcnc_sub(&name) {
                            self.run(s + 1, Some(&name), depth + 1);
                        } else {
                            self.err(b.line, format!("o{name} is not defined"));
                        }
                    }
                    _ => {}
                }
                i += 1;
                continue;
            }

            if let Flow::Return = self.fanuc(b, sub_name, depth) {
                return;
            }
            i += 1;
        }
        if let Some(name) = sub_name {
            let line = self.blocks.last().map_or(0, |b| b.line);
            self.err(line, format!("o{name} sub has no endsub"));
        }
    }

    /// Handle M98 / M99 on an ordinary block, emitting the block itself.
    fn fanuc(&mut self, b: &Block, sub_name: Option<&str>, depth: usize) -> Flow {
        let has_m = |n: f64| b.all('M').any(|m| (m - n).abs() < 1e-9);
        let call = has_m(98.0);
        let ret = has_m(99.0);
        if depth == 0 && (has_m(30.0) || has_m(2.0)) {
            // End of the main program; anything after it is subprograms.
            self.out.push(b.clone());
            return Flow::Return;
        }
        if !call && !ret {
            self.out.push(b.clone());
            return Flow::Continue;
        }

        // Keep everything except the call/return words.
        let mut rest = b.clone();
        rest.words.retain(|w| {
            !(w.letter == 'M' && (w.value == 98.0 || w.value == 99.0))
                && !(call && matches!(w.letter, 'P' | 'L' | 'K'))
        });
        rest.oword = None;
        if !rest.words.is_empty() {
            self.out.push(rest);
        }

        if call {
            let Some(p) = b.get('P') else {
                self.err(b.line, "M98 without a P program number".into());
                return Flow::Continue;
            };
            let p = p.round() as u64;
            let explicit = b.get('L').or_else(|| b.get('K'));
            let (count, program) = match explicit {
                Some(n) => (n.round().max(0.0) as u64, p),
                None if p > 9_999 => (p / 10_000, p % 10_000),
                None => (1, p),
            };
            let count = count.max(1);
            let number = program.to_string();
            match self.find_fanuc_program(&number) {
                None => self.err(
                    b.line,
                    format!("program O{number} called by M98 is not in this file"),
                ),
                Some(_) if depth >= MAX_DEPTH => {
                    self.err(b.line, format!("call nesting deeper than {MAX_DEPTH}"))
                }
                Some(start) => {
                    for _ in 0..count {
                        self.run_fanuc(start + 1, depth + 1);
                    }
                }
            }
        }

        if ret && (depth > 0 || sub_name.is_none()) {
            // M99 ends a Fanuc sub; in the main program it ends the run.
            if depth == 0 {
                let mut end = b.clone();
                end.words = vec![crate::gcode::Word {
                    letter: 'M',
                    value: 30.0,
                }];
                end.oword = None;
                self.out.push(end);
            }
            return Flow::Return;
        }
        Flow::Continue
    }

    fn run_fanuc(&mut self, start: usize, depth: usize) {
        let mut i = start;
        while i < self.blocks.len() {
            let b = &self.blocks[i];
            if b.oword.as_ref().is_some_and(|o| o.keyword.is_none()) {
                // Next program header without M99: treat as end of this one.
                self.err(
                    b.line,
                    "subprogram ran into the next program without M99".into(),
                );
                return;
            }
            if let Flow::Return = self.fanuc(b, None, depth) {
                return;
            }
            i += 1;
        }
        let line = self.blocks.last().map_or(0, |b| b.line);
        self.err(line, "subprogram has no M99".into());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcode::parse_program;

    fn lines(src: &str) -> (Vec<usize>, Vec<SubError>) {
        let (b, e) = expand(&parse_program(src).unwrap());
        (b.iter().map(|b| b.line).collect(), e)
    }

    #[test]
    fn linuxcnc_sub_call() {
        let src = "o100 sub\nG1 X1\no100 endsub\nG0 X0\no100 call\no100 call\nM2\n";
        let (l, e) = lines(src);
        assert!(e.is_empty(), "{e:?}");
        assert_eq!(l, vec![4, 2, 2, 7]);
    }

    #[test]
    fn fanuc_m98_with_repeats() {
        let src = "O1000\nG0 X0\nM98 P2000 L2\nM98 P00032000\nM30\nO2000\nG1 X1\nM99\n";
        let (l, e) = lines(src);
        assert!(e.is_empty(), "{e:?}");
        // O1000 header block is kept (no words), then 2 + 3 calls of line 7.
        assert_eq!(l, vec![1, 2, 7, 7, 7, 7, 7, 5]);
    }

    #[test]
    fn missing_sub_and_recursion() {
        let (_, e) = lines("o5 call\n");
        assert!(e[0].msg.contains("not defined"));
        let (_, e) = lines("O1\nM98 P2\nM30\nO2\nM98 P2\nM99\n");
        assert!(e.iter().any(|e| e.msg.contains("nesting")));
    }
}
