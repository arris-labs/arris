# Conducting a plan in Codex

- Give each fresh agent the numbered step, its acceptance commands, the
  applicable rule and ADR sections, and the relevant source and fixture
  paths. Include this reference in the handoff. Start with those sections
  and follow references as needed. Use targeted reads and excerpts so
  repeated searches and tool output stay small.
- Use the terminal agent result as the handback. Give the commit SHA,
  relevant checks and their outcomes, and any verification still running.
  Record the pinned SHA, expected finish, job ID or exit-status path, and
  log path in the agent's log at launch. Send an interim message only when
  the conductor needs to make a decision or take action.
  The conductor waits on the agent with an interruptible `wait_agent`
  timeout appropriate to the work. Reuse the existing job result or log
  when checking a command that was started already.
- Treat a terminal agent error as a handback that needs action. Check HEAD,
  the staged and unstaged diff, and any existing jobs before retrying. If
  the step is incomplete and the harness permits, use the existing
  one-retry rule in *Conduct*, giving the replacement the current state
  and the pending checks. If the harness blocks the retry, report the
  actual error, the last completed step, and what is staged or still
  running so the next turn can resume.
- When a step introduces singular topology or tolerance-based
  classification, check the named acceptance against inputs on both sides
  of the threshold and downstream operations that should build or return
  a typed refusal. Surface gaps in the plan before handback and apply
  *Keep to* if filling a gap would change the plan's acceptance.
- When editing a sweep recipe, include
  `cargo nextest run -p arris --test corpus_lint` in the step checks.
  For a recipe that ends in a typed refusal, record the exact refusal in
  the recipe and exercise it through the corpus runner.
