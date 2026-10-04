# ADR-0046 — An oracle's result its operands contradict is a refusal, not a disagreement

- Status: accepted (2026-10-04)
- Plan: `nightly-failures` step 6 (⚠ OPEN: a differential convention)
- Amends: ADR-0024 §2 (every outcome of the differential is classed) for the
  one class of case where the oracle, not Arris, gives the wrong answer

## Context

Three of the nightly differential's failures (2026-10-01 cases 335, 696, 856)
were Open CASCADE's boolean answering wrongly for operands it had built:
twice it recorded no solid for a fuse of two revolves and for a common of a
fuse and a third revolve, and once its fuse of an extrusion and a mirrored
revolve returned a solid of exactly the extrusion's volume, 1781.91, below
the revolve's own 12620.20 — a fuse that dropped an operand. Arris's results
for the three are valid at `Full`, and the identity
`V(A ∪ B) + V(A ∩ B) = V(A) + V(B)` holds over a fresh union and common of
the same operands to better than 1e-9 relative, as does the result's own
volume against the one operation it is. The differential called each a
`Disagree` and failed the run, though Arris is the one that is right.

## Decision

**1. No solid from the oracle.** Where `expected.json` records no solid and
Arris builds a body, the recipe's result is held to itself: the checker green
at `Full`, and, when the result is a `fuse`, `common` or `cut` of two
operands, the identity above to `ADDITIVE_REL` (1e-6 relative, a bound the
identity has held far inside), the result's volume the one its operation
gives (`V(A ∪ B)`, `V(A ∩ B)`, `V(A) − V(A ∩ B)`), an empty common counting
as nothing. Held, it is an `OracleRefuses`, counted under its reason in the
histogram; not held, or the result no boolean of two operands, it is the
`Disagree(build)` it was.

**2. A volume the operands bound.** Where the oracle did record a solid and
its volume is outside what the operands allow whatever the kernel — below the
larger operand or above the two together for a fuse, above the smaller for a
common, outside `V(A) − V(B)` to `V(A)` for a cut — and Arris's result holds
as in 1, that is an `OracleRefuses` too. A volume inside the bounds is
compared as before; the bounds only recognise an oracle that is impossible,
never one that merely differs.

**3. The operands are Arris's.** The bounds use Arris's operand volumes,
which the corpus holds to the oracle's on every fixture that builds them; a
fault in an operand's build shows there first, and a result built from a
wrong operand would fail the additivity of 1.

## Consequences

- No Arris failure is hidden: both rules need Arris's body valid and
  additive, so a body Open CASCADE could not build and Arris builds
  wrongly still fails, now with the identity's numbers in the message.
- The cases are counted `OracleRefuses`, so the histogram says how often
  Open CASCADE failed this way. A reading of the oracle's failures worth
  keeping stays in the run's report, not in a fixture: no fixture can hold
  Arris to an answer the oracle does not give.
- `differential::judge` has two new private helpers and a constant;
  nothing public changes.

## Alternatives considered

- **A named `EXCLUSIONS` entry on the `Disagree(build)` text.** Rejected:
  it also covers an Arris body built where nothing should be, which is a
  real defect, and an exclusion waits on a fixture the fix retires, where
  here nothing in Arris is to be fixed.
- **Hold the oracle's answer to the closed form.** Rejected: a drawn
  recipe has none; additivity needs no closed form and is what the boolean
  property tests already hold.
