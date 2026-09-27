# Fuzz targets over the intersectors, the STEP reader and body bytes

ADR-0024 §5, ADR-0025, ADR-0029. The crate sits outside the workspace (`exclude = ["fuzz"]`).
It builds on the nightly toolchain under `cargo fuzz` and is never
published. Setup, once:

```sh
rustup toolchain install nightly --profile minimal
cargo install cargo-fuzz --locked
```

| Target | Operands | Asserts |
|---|---|---|
| `intersect_surfaces` | two analytic surfaces in any pose, in a cube about the origin | no panic; every curve and point of the answer on both surfaces within the tolerance; the same answer twice |
| `intersect_curve_surface` | a line, a conic, a NURBS curve or a fitted section, against an analytic surface | no panic; every hit on both; a bounded `Coincident` curve on the surface all along; the same answer twice |
| `intersect_curves` | two of those curves | no panic; every hit on both; a bounded `Coincident` curve on the other where its projection is exact; the same answer twice |
| `step_read` | any text, bytes that are not UTF-8 replaced | no panic in `step::read`; the same answer twice, read into two fresh models; a parse error's line and column inside the text. Seeded with every solid fixture's STEP, Arris's and Open CASCADE's; not Open CASCADE's B-spline conversions, which take tens of seconds each with debug assertions on and which the corpus runner reads already. A crash is shrunk to a test in `crates/arris-io/tests/step_read.rs` |
| `body_read` | any bytes to `body::read`, and to `body::from_json` where they are UTF-8, into a model holding the guard's tetrahedron | no panic; the same answer twice; a body read checker-green at `Level::Full`; a refusal leaving the model's native bytes as they were. Seeded with every file of body bytes' guard, `crates/arris-io/tests/body/v<N>/`. A crash is shrunk to a test in `crates/arris-io/tests/body.rs` |

The intersector targets' byte layout, the folding of a number into its range and what makes an
input skipped rather than a finding are documented in `src/lib.rs`. The
tolerance is the kernel's default.

```sh
cargo run --manifest-path fuzz/Cargo.toml --example seed    # the seed corpus: tests/fixtures/geom/ pairs, every solid fixture's STEP, body bytes' guard
cd fuzz
cargo +nightly fuzz run -s none intersect_surfaces -- -max_total_time=60
cargo run --example show -- intersect_surfaces artifacts/intersect_surfaces/crash-…
```

The targets run with `-s none`, without AddressSanitizer. The kernel
crates are `forbid(unsafe_code)`, so ASan has nothing of theirs to find,
and it costs about twentyfold in executions per second. Debug assertions
stay on.

A `step_read` input is a text file already: a crash is read as it is
(`cargo run -p arris-debug --example inspect_step -- <file>`), and shrunk
to a parser unit test in `crates/arris-io/src/step/part21.rs` or a
reader test in `crates/arris-io/tests/step_read.rs`.

`show` prints what an input decodes to and what the intersector answers.
It is the first look at a crash before the crash is shrunk
(`cargo fuzz tmin`) into a `geometry` fixture under
`tests/fixtures/regression/`, as `tests/fixtures/README.md` §Property-test
failures describes.
