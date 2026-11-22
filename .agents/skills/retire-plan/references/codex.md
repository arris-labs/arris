# Retiring a plan in Codex

- Use the [Codex conduct guidance](../../work/references/codex.md) for
  agent notifications and terminal errors. Give each fix agent the
  finding, affected commit, relevant plan and ADR sections, and the
  fixture and source paths it needs. Give the review agent the full plan
  and commit list for its cross-step review.
- Record verification and acceptance job details at launch in the
  coordinator's log.
  Give the conductor the pinned SHA and final results in the retirement
  handback. Send an interim message when the conductor needs to make a
  decision or take action. Within retirement, rely on the agent's terminal
  result for review and fix handbacks.
- Start each acceptance run once for the pinned SHA. While it runs, do the
  independent retirement work listed in the skill. Then wait on the
  existing job through the runner. Read the relevant log excerpt on
  failure or when the job misses its expected finish. If the turn resumes,
  recover the pinned SHA and existing job results before launching a run.
- In the review brief for singular topology or tolerance changes, ask
  which fixtures exercise both sides of the threshold and which downstream
  operations prove the resulting topology can be used or fails with the
  intended typed error.
