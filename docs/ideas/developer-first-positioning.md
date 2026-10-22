# Idea: developer-first-positioning

- Status: Accepted 2026-10-17 (A, L-A); credibility, ADR-0067 and the script corpus landed; the modelling layer's first milestone absorbed by plans/modelling-layer (2026-10-22). Still to be absorbed: the MCP server
- Raised: 2026-10-17
- Prompt (verbatim from the human): "I think that positioning arris target audience as DIY enthusiasts was wrong. Read @docs/notes/positioning.md - do you think that direction is bettter for arris?" — then, on the verdict: "I agree with you on all points, so yes, /idea that"

The positioning, the layer's place and shape, the script corpus and the
launch gate are settled in ADR-0067. The full brainstorm, including the
script corpus's friction list, is in git history (commit 76d42508). What
is left here is the one part no plan has absorbed yet.

## The MCP server

A thin Python package over the wheel, installed as an optional extra
(`pip install arris[mcp]`). It runs an `arris.build` script and returns the
body's measures, the checker's report, and any typed refusal naming its
entities, so an agent can correct its script and export a STEP file that
has been checked. About 3 steps, planned after `plans/modelling-layer`
retires, because it calls the layer and breaks with it.

It returns no rendered image: ADR-0034 §5 keeps PNG out of the wheel, and
adding one is its own amendment. The JS binding is ranked after it as a
backlog line. The playground is rejected for this repository
(`docs/BACKLOG.md` §Rejected).

## Decision

Q9 (2026-10-17), as given: "Accept … Q9=A". The MCP server is
`arris[mcp]` in this repository, planned after the layer's first
milestone, and returns no rendered image under ADR-0034 §5.
