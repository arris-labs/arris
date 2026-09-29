//! A STEP file seen as the `inspect` skill sees a shape: every solid of it
//! read by `arris_io::step::read` — an assembly's at each placement —
//! with its text dump, its checker report at `Full` and a rendered PNG,
//! or the refusal naming the file entity where the reader stopped.

use crate::unmetered::step_read;
use std::path::{Path, PathBuf};

use arris_io::arris_check::arris_topo::Model;
use arris_io::arris_check::arris_topo::provenance::FileEntity;
use arris_io::arris_check::{Level, Report, check};
use arris_io::step::{ReadError, ReadOptions, Refusal};

use crate::{View, dump_text, render_body};

/// One solid of a file, seen.
#[derive(Debug)]
pub struct Seen {
    /// The solid's file entity and placement.
    pub entity: FileEntity,
    /// What was seen of it, or why the reader refused it.
    pub result: Result<Sight, Refusal>,
}

/// What a solid read looks like.
#[derive(Debug)]
pub struct Sight {
    /// Its [`dump_text`].
    pub dump: String,
    /// The checker at `Full`.
    pub report: Report,
    /// Its isometric render, `target/inspect/<name>-<id>-<instance>.png`,
    /// or why there is none.
    pub png: Result<PathBuf, String>,
}

/// Why a file could not be seen at all.
#[derive(Debug, thiserror::Error)]
pub enum InspectError {
    /// The file could not be read from disk.
    #[error("{path}: {source}")]
    Io {
        /// The file.
        path: PathBuf,
        /// The cause.
        source: std::io::Error,
    },
    /// The text is not a Part 21 exchange structure.
    #[error(transparent)]
    Read(#[from] ReadError),
}

/// Every solid of the STEP file at `path` read into `model` in
/// millimetres, each dumped, checked at `Full` and rendered to a PNG named
/// after `name`, its entity and its placement — or its refusal. The
/// results are the reader's, in its order.
///
/// Errors: [`InspectError::Io`] for a file that does not read from disk,
/// [`InspectError::Read`] for one that does not parse.
///
/// ```no_run
/// use arris_debug::step_file;
/// use arris_io::arris_check::arris_topo::Model;
///
/// let mut m = Model::default();
/// for seen in step_file::inspect(&mut m, "part.step".as_ref(), "part").unwrap() {
///     match seen.result {
///         Ok(sight) => println!("{:?}\n{}\n{}", seen.entity, sight.report, sight.dump),
///         Err(refusal) => println!("{:?}: {refusal}", seen.entity),
///     }
/// }
/// ```
pub fn inspect(model: &mut Model, path: &Path, name: &str) -> Result<Vec<Seen>, InspectError> {
    let text = std::fs::read_to_string(path).map_err(|source| InspectError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let read = step_read(model, &text, &ReadOptions::default())?;
    Ok(read
        .solids
        .into_iter()
        .map(|solid| {
            let entity = solid.entity;
            let result = solid.result.map(|back| {
                let tag = format!("{name}-{}-{}", entity.id, entity.instance);
                Sight {
                    dump: dump_text(model, back.body).unwrap_or_else(|e| e.to_string()),
                    report: check(model, back.body, Level::Full),
                    png: render_body(model, back.body, View::Iso, None, tag)
                        .map_err(|e| e.to_string()),
                }
            });
            Seen { entity, result }
        })
        .collect())
}
