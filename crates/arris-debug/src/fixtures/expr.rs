//! The arithmetic a recipe number may be written in: the same grammar the
//! oracle's `recipe.py` evaluates, so `"50 + R * cos(radians(45))"` means
//! one thing on both sides.

use std::collections::BTreeMap;

/// Why an expression did not evaluate.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ExprError {
    /// A character that is not part of the grammar.
    #[error("unexpected {found:?} at {at} in {text:?}")]
    Unexpected {
        /// The text.
        text: String,
        /// Byte offset.
        at: usize,
        /// What was there.
        found: String,
    },
    /// A name that is neither a parameter nor a constant.
    #[error("unknown name {name:?} in {text:?}")]
    UnknownName {
        /// The text.
        text: String,
        /// The name.
        name: String,
    },
    /// A function that is not in the table, or called with the wrong arity.
    #[error("unknown function {name:?}/{arity} in {text:?}")]
    UnknownFunction {
        /// The text.
        text: String,
        /// The name.
        name: String,
        /// How many arguments it was given.
        arity: usize,
    },
}

/// Evaluates `text` over `params`. Grammar: `+ - * / ^`, unary `-`,
/// parentheses, decimal numbers, names from `params` and `pi`, and the
/// functions `sin cos tan atan sqrt radians degrees abs` of one argument.
pub fn eval(text: &str, params: &BTreeMap<String, f64>) -> Result<f64, ExprError> {
    let mut p = Parser {
        text,
        pos: 0,
        params,
    };
    let v = p.expr()?;
    p.skip_ws();
    if p.pos != text.len() {
        return Err(p.unexpected());
    }
    Ok(v)
}

struct Parser<'a> {
    text: &'a str,
    pos: usize,
    params: &'a BTreeMap<String, f64>,
}

impl Parser<'_> {
    fn unexpected(&self) -> ExprError {
        let found = self.text[self.pos..]
            .chars()
            .next()
            .map_or(String::from("end"), |c| c.to_string());
        ExprError::Unexpected {
            text: self.text.to_string(),
            at: self.pos,
            found,
        }
    }

    fn skip_ws(&mut self) {
        while self.text[self.pos..].starts_with(' ') {
            self.pos += 1;
        }
    }

    fn peek(&mut self) -> Option<char> {
        self.skip_ws();
        self.text[self.pos..].chars().next()
    }

    fn expr(&mut self) -> Result<f64, ExprError> {
        let mut v = self.term()?;
        loop {
            match self.peek() {
                Some('+') => {
                    self.pos += 1;
                    v += self.term()?;
                }
                Some('-') => {
                    self.pos += 1;
                    v -= self.term()?;
                }
                _ => return Ok(v),
            }
        }
    }

    fn term(&mut self) -> Result<f64, ExprError> {
        let mut v = self.unary()?;
        loop {
            match self.peek() {
                Some('*') => {
                    self.pos += 1;
                    v *= self.unary()?;
                }
                Some('/') => {
                    self.pos += 1;
                    v /= self.unary()?;
                }
                _ => return Ok(v),
            }
        }
    }

    fn unary(&mut self) -> Result<f64, ExprError> {
        match self.peek() {
            Some('-') => {
                self.pos += 1;
                Ok(-self.unary()?)
            }
            Some('+') => {
                self.pos += 1;
                self.unary()
            }
            _ => self.power(),
        }
    }

    fn power(&mut self) -> Result<f64, ExprError> {
        let base = self.atom()?;
        if self.peek() == Some('^') {
            self.pos += 1;
            let exp = self.unary()?;
            return Ok(base.powf(exp));
        }
        Ok(base)
    }

    fn atom(&mut self) -> Result<f64, ExprError> {
        match self.peek() {
            Some('(') => {
                self.pos += 1;
                let v = self.expr()?;
                if self.peek() != Some(')') {
                    return Err(self.unexpected());
                }
                self.pos += 1;
                Ok(v)
            }
            Some(c) if c.is_ascii_digit() || c == '.' => self.number(),
            Some(c) if c.is_ascii_alphabetic() || c == '_' => self.name(),
            _ => Err(self.unexpected()),
        }
    }

    fn number(&mut self) -> Result<f64, ExprError> {
        let start = self.pos;
        let bytes = self.text.as_bytes();
        while self.pos < bytes.len()
            && (bytes[self.pos].is_ascii_digit() || bytes[self.pos] == b'.')
        {
            self.pos += 1;
        }
        if self.pos < bytes.len() && (bytes[self.pos] == b'e' || bytes[self.pos] == b'E') {
            let save = self.pos;
            self.pos += 1;
            if self.pos < bytes.len() && (bytes[self.pos] == b'+' || bytes[self.pos] == b'-') {
                self.pos += 1;
            }
            let digits = self.pos;
            while self.pos < bytes.len() && bytes[self.pos].is_ascii_digit() {
                self.pos += 1;
            }
            if self.pos == digits {
                self.pos = save;
            }
        }
        self.text[start..self.pos]
            .parse()
            .map_err(|_| ExprError::Unexpected {
                text: self.text.to_string(),
                at: start,
                found: self.text[start..self.pos].to_string(),
            })
    }

    fn name(&mut self) -> Result<f64, ExprError> {
        let start = self.pos;
        let bytes = self.text.as_bytes();
        while self.pos < bytes.len()
            && (bytes[self.pos].is_ascii_alphanumeric() || bytes[self.pos] == b'_')
        {
            self.pos += 1;
        }
        let name = &self.text[start..self.pos];
        if self.peek() == Some('(') {
            self.pos += 1;
            let mut args = Vec::new();
            if self.peek() != Some(')') {
                loop {
                    args.push(self.expr()?);
                    match self.peek() {
                        Some(',') => self.pos += 1,
                        Some(')') => break,
                        _ => return Err(self.unexpected()),
                    }
                }
            }
            self.pos += 1;
            let f: fn(f64) -> f64 = match (name, args.len()) {
                ("sin", 1) => f64::sin,
                ("cos", 1) => f64::cos,
                ("tan", 1) => f64::tan,
                ("atan", 1) => f64::atan,
                ("sqrt", 1) => f64::sqrt,
                ("radians", 1) => f64::to_radians,
                ("degrees", 1) => f64::to_degrees,
                ("abs", 1) => f64::abs,
                _ => {
                    return Err(ExprError::UnknownFunction {
                        text: self.text.to_string(),
                        name: name.to_string(),
                        arity: args.len(),
                    });
                }
            };
            return Ok(f(args[0]));
        }
        if let Some(v) = self.params.get(name) {
            return Ok(*v);
        }
        if name == "pi" {
            return Ok(core::f64::consts::PI);
        }
        Err(ExprError::UnknownName {
            text: self.text.to_string(),
            name: name.to_string(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> BTreeMap<String, f64> {
        [("R", 35.0), ("r", 3.0), ("t", 10.0)]
            .into_iter()
            .map(|(k, v)| (k.to_string(), v))
            .collect()
    }

    #[test]
    fn evaluates_the_recipe_expressions() {
        let p = params();
        assert_eq!(eval("50 + R * cos(radians(0))", &p), Ok(85.0));
        assert!(
            (eval("50 + R * cos(radians(45))", &p).unwrap() - (50.0 + 35.0 * 0.5_f64.sqrt())).abs()
                < 1e-12
        );
        assert_eq!(
            eval("100 * 100 * t - 8 * pi * r * r * t", &p),
            Ok(100000.0 - 8.0 * core::f64::consts::PI * 90.0)
        );
        assert_eq!(eval("-t + 2", &p), Ok(-8.0));
        assert_eq!(eval("2 ^ 3 ^ 1", &p), Ok(8.0));
        assert_eq!(eval("(1 + 2) * 3", &p), Ok(9.0));
        assert_eq!(eval("1e-3 * 1000", &p), Ok(1.0));
        assert_eq!(eval("sqrt(16) / 2", &p), Ok(2.0));
        assert_eq!(eval("4 * atan(1)", &p), Ok(core::f64::consts::PI));
    }

    #[test]
    fn errors_name_the_problem() {
        let p = params();
        assert!(
            matches!(eval("x + 1", &p), Err(ExprError::UnknownName { name, .. }) if name == "x")
        );
        assert!(
            matches!(eval("foo(1)", &p), Err(ExprError::UnknownFunction { name, arity: 1, .. }) if name == "foo")
        );
        assert!(matches!(eval("1 +", &p), Err(ExprError::Unexpected { .. })));
        assert!(matches!(eval("(1", &p), Err(ExprError::Unexpected { .. })));
        assert!(matches!(
            eval("1 2", &p),
            Err(ExprError::Unexpected { at: 2, .. })
        ));
    }

    #[derive(serde::Deserialize)]
    struct Case {
        expr: String,
        params: BTreeMap<String, f64>,
        #[serde(default)]
        expect: Option<f64>,
        #[serde(default)]
        error: bool,
    }

    /// `tests/fixtures/expr-cases.json`: the same grammar cases the
    /// oracle's `selftest.py` evaluates too, so `^`, `**` and the rest
    /// mean the same thing on both sides.
    #[test]
    fn the_expression_cases_both_sides_evaluate() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../tests/fixtures/expr-cases.json");
        let text = std::fs::read_to_string(&path).unwrap();
        let cases: Vec<Case> = serde_json::from_str(&text).unwrap();
        assert!(!cases.is_empty());
        for case in cases {
            let got = eval(&case.expr, &case.params);
            if case.error {
                assert!(
                    got.is_err(),
                    "{:?}: expected an error, got {got:?}",
                    case.expr
                );
                continue;
            }
            let expect = case.expect.expect("a non-error case names `expect`");
            let got = got.unwrap_or_else(|e| panic!("{:?}: {e}", case.expr));
            assert!(
                (got - expect).abs() <= 1e-6,
                "{:?}: {got} != {expect}",
                case.expr
            );
        }
    }
}
