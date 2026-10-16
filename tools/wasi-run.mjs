// Cargo's runner for `wasm32-wasip1` test binaries: Node's WASI preview 1,
// so `cargo test --target wasm32-wasip1` runs the determinism corpus on
// wasm32 with no tool beyond Node. Set it as
//   CARGO_TARGET_WASM32_WASIP1_RUNNER="node tools/wasi-run.mjs"
// No directory is preopened: a test run here embeds what it reads.
import { readFile } from "node:fs/promises";
import { argv, env, exit } from "node:process";
import { WASI } from "node:wasi";

const [wasmPath, ...args] = argv.slice(2);
const wasi = new WASI({ version: "preview1", args: [wasmPath, ...args], env, returnOnExit: true });
const module = await WebAssembly.compile(await readFile(wasmPath));
const instance = await WebAssembly.instantiate(module, wasi.getImportObject());
exit(wasi.start(instance));
