//! The markdown table the corpus reports write: a header, the alignment
//! rule and one line per row, in the one shape every report shares.

use std::fmt;

/// How a column's cells sit: the rule line writes `---` or `---:`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Align {
    Left,
    Right,
}

/// A table of text cells: [`Table::new`] with the columns, [`Table::row`]
/// per line, then `Display` (or `to_string`) writes it.
///
/// A row must hold as many cells as the table has columns; a shorter or
/// longer one is the writer's bug, caught by a debug assertion.
#[derive(Debug)]
pub(crate) struct Table {
    columns: Vec<(String, Align)>,
    rows: Vec<Vec<String>>,
}

impl Table {
    /// An empty table with these `(header, alignment)` columns.
    pub(crate) fn new<H: Into<String>>(columns: impl IntoIterator<Item = (H, Align)>) -> Table {
        Table {
            columns: columns.into_iter().map(|(h, a)| (h.into(), a)).collect(),
            rows: Vec::new(),
        }
    }

    /// Appends a row of cells, one per column.
    pub(crate) fn row<C: Into<String>>(&mut self, cells: impl IntoIterator<Item = C>) {
        let cells: Vec<String> = cells.into_iter().map(Into::into).collect();
        debug_assert_eq!(
            cells.len(),
            self.columns.len(),
            "a row has a cell per column"
        );
        self.rows.push(cells);
    }
}

impl fmt::Display for Table {
    /// The header line, the rule (`|---|---:|`) and the rows, each ending
    /// in a newline.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("|")?;
        for (header, _) in &self.columns {
            write!(f, " {header} |")?;
        }
        f.write_str("\n|")?;
        for (_, align) in &self.columns {
            f.write_str(match align {
                Align::Left => "---|",
                Align::Right => "---:|",
            })?;
        }
        f.write_str("\n")?;
        for row in &self.rows {
            f.write_str("|")?;
            for cell in row {
                write!(f, " {cell} |")?;
            }
            f.write_str("\n")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_table_writes_its_header_rule_and_rows() {
        let mut t = Table::new([("part", Align::Left), ("edges", Align::Right)]);
        t.row(["plate", "3"]);
        t.row(["bracket".to_string(), 12.to_string()]);
        assert_eq!(
            t.to_string(),
            "| part | edges |\n|---|---:|\n| plate | 3 |\n| bracket | 12 |\n"
        );
    }

    #[test]
    fn a_table_without_rows_is_its_header_and_rule() {
        let t = Table::new([("a", Align::Right)]);
        assert_eq!(t.to_string(), "| a |\n|---:|\n");
    }
}
