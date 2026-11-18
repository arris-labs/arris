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

Follow [Contributing from a fork](AGENTS.md#contributing-from-a-fork) to
set up your clone, prepare your topic branch, and open a pull request.
Run [`tools/pr-check.sh`](tools/pr-check.sh) on your committed branch
before submitting it. The check ensures the tip has a local verification
pass for the push hook. The first run in a fresh clone may run a wider set
of tests to establish that pass.

Read the [kernel, git, and docs rules](.agents/rules/) and the
[provenance policy](PROVENANCE.md) before changing code. Include a fixture
and its Open CASCADE oracle value (`tools/oracle/`) when adding behaviour.
For anything larger than a fix, open a discussion first. Check
`docs/adr/` for decisions that already apply.

## Licence

Unless you say otherwise, any contribution you submit for inclusion in
arris is dual-licensed as MIT or Apache-2.0, as the project is, without
any additional terms or conditions.
