# The script corpus

Each part is a script (`part.py`, on the raw binding `arris.Model`) that the
harness runs stage by stage against its Open CASCADE checkpoints
(`expected.json`) and its recorded outcome (`outcome.json`); see
`harness.py` and `tests/test_script_corpus.py`.

Four parts are also ported to the modelling layer as `build.py` (`arris.build`,
ADR-0067): the harness runs both scripts against the same `oracle.json`,
`expected.json` and `outcome.json`, and the test holds the two to the same
outcomes, the same checkpoints and, for the tagged parts, the same tag on
every face.

| Part | Layer used | `part.py` | `build.py` | Model after the run: raw | layer |
|---|---|---|---|---|---|
| `enclosure` | selection (`parallel`, `extreme`, `within`), `shell`, `boss`, `hole` | 48 lines | 27 | 643 entities | 639 |
| `spur-gear-blank` | polar patterns, `pocket` for the bore, holes and keyway | 51 | 37 | 2894 | 3400 |
| `cantilever` | tags through a cut and a fillet | 41 | 24 | 199 | 199 |
| `duct-obstacle` | tags through a through-hole and a blind port | 36 | 30 | 143 | 143 |

Lines are non-blank, non-comment lines of the script, its docstring and
constants included. The model's size is the entities `Model.retain([])`
frees after the run: the layer makes no model grow that the raw script's
own tools do not, except the gear's, where a pattern's sketches are
extruded into one tool each instead of a tool turned from one (3400
entities against 2894).
