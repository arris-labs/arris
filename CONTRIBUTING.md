# Contributing

## Reports are the most useful contribution

A script that makes arris refuse, panic or return a wrong body is worth
more than any patch. Open an issue with the *Refusal or wrong result*
template: the shortest script that shows it, the full error, and what you
expected. A report that reproduces becomes a fixture in the test corpus,
held to values computed by Open CASCADE, and stays there once fixed.

A typed refusal is arris working as designed, but it is still worth
reporting: the refusals people hit decide what gets built next.

Questions, "is arris a fit for…" and ideas go to
[Discussions](https://github.com/arris-labs/arris/discussions).

## Code

Arris is developed under written rules that every change follows. Read
them before a pull request:

- [AGENTS.md](AGENTS.md): the project in one page, its setup, and where the
  rest lives;
- [.agents/rules/](.agents/rules/): the kernel rules (no panics on
  geometry, provenance from every operation, determinism, no tolerance
  literals), git, and the docs lifecycle;
- [PROVENANCE.md](PROVENANCE.md): code from other kernels is read, never
  copied.

Set up the hooks once per clone (`git config core.hooksPath .githooks`); the
pre-commit hook runs fmt and clippy, and the tests the change reaches run
after the commit, to be verified before a push (ADR-0078). A change
that adds behaviour comes with a fixture and its Open CASCADE oracle value
(`tools/oracle/`). For anything larger than a fix, open a discussion first:
the design may already be settled in `docs/adr/`.

## Licence

Unless you say otherwise, any contribution you submit for inclusion in
arris is dual-licensed as MIT or Apache-2.0, as the project is, without
any additional terms or conditions.
