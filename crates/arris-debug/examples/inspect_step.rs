//! `cargo run -p arris-debug --example inspect_step -- [--max-tolerance T]
//! <file.step> [name]`: every solid of a STEP file read, its checker
//! report at `Full` and its text dump printed, its isometric render written
//! under `target/inspect/` — or its refusal (`arris_debug::step_file`).
//! `--max-tolerance` reads into a model of that ceiling, as the gap census
//! does (ADR-0058).

use arris_debug::step_file;
use arris_math::Precision;
use arris_topo::Model;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1).peekable();
    let mut precision = Precision::DEFAULT;
    if args.peek().map(String::as_str) == Some("--max-tolerance") {
        args.next();
        precision.max_tolerance = args
            .next()
            .ok_or("--max-tolerance needs a value")?
            .parse()?;
    }
    let Some(path) = args.next() else {
        eprintln!("usage: inspect_step [--max-tolerance T] <file.step> [name]");
        std::process::exit(2);
    };
    let name = args.next().unwrap_or_else(|| "step".into());
    let mut model = Model::new(precision)?;
    for seen in step_file::inspect(&mut model, path.as_ref(), &name)? {
        println!("== #{} [{}]", seen.entity.id, seen.entity.instance);
        match seen.result {
            Ok(sight) => {
                println!("{}", sight.report);
                match sight.png {
                    Ok(png) => println!("render: {}", png.display()),
                    Err(e) => println!("render: {e}"),
                }
                println!("{}", sight.dump);
            }
            Err(refusal) => println!("refused ({}): {refusal}", refusal.kind()),
        }
    }
    Ok(())
}
