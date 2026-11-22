"""Regression checks for plan-scoped Claude Code and Codex digests."""

import contextlib
import io
import json
from pathlib import Path
import runpy
import sys
import tempfile
import unittest
from unittest.mock import patch


DIGEST = runpy.run_path(str(Path(__file__).with_name("session-digest")))
STAMP = "2026-11-21T20:00:00Z"
SLUG = "tight-body-bounds"
PLAN = f"docs/plans/{SLUG}.md"


def codex(kind, **fields):
    return {
        "timestamp": STAMP,
        "type": "response_item",
        "payload": {"type": kind, **fields},
    }


def claude(role, content, **fields):
    return {"timestamp": STAMP, "type": role, "message": {"content": content}, **fields}


def write_session(directory, name, *rows):
    path = Path(directory) / f"{name}.jsonl"
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    return str(path)


class PlanSelectionTest(unittest.TestCase):
    def matches(self, *rows):
        with tempfile.TemporaryDirectory() as directory:
            path = write_session(directory, "test", *rows)
            return DIGEST["plan_session"](path, SLUG)

    def test_explicit_plan_invocations(self):
        self.assertTrue(self.matches(claude(
            "user", f"<command-name>/plan</command-name><command-args>{SLUG}</command-args>"
        )))
        self.assertTrue(self.matches(codex(
            "message", role="user", content=[{"type": "input_text", "text": f"$work {SLUG} step 2"}]
        )))

    def test_exact_plan_path_in_tool_inputs(self):
        self.assertTrue(self.matches(codex(
            "custom_tool_call", input=f"*** Update File: {PLAN}"
        )))
        self.assertTrue(self.matches(claude("assistant", [{
            "type": "tool_use", "name": "Read", "input": {"file_path": f"/checkout/{PLAN}"}
        }])))
        self.assertFalse(self.matches(codex("function_call", arguments=f"cat {PLAN}.bak")))

    def test_inherited_context_and_tool_results(self):
        self.assertFalse(self.matches(
            codex("message", role="user", content=[
                {"type": "input_text", "text": f"Instructions mention {PLAN}."}
            ]),
            codex("function_call_output", output=f"cat {PLAN}"),
            claude("user", f"/work {SLUG}", isMeta=True),
            claude("user", [{"type": "tool_result", "content": PLAN}]),
        ))

    def test_retro_turn_does_not_select_itself(self):
        retro = codex(
            "message", role="user", content=[{"type": "input_text", "text": f"$retro {SLUG}"}]
        )
        read = codex("function_call", arguments=f"git log -- {PLAN}")
        self.assertFalse(self.matches(retro, read))
        self.assertTrue(self.matches(
            retro, read, claude("user", "/work step 2"), read
        ))

    def test_cli_keeps_descendants_and_offers_broad_matching(self):
        parent = "11111111-1111-1111-1111-111111111111"
        child = "22222222-2222-2222-2222-222222222222"
        incidental = "33333333-3333-3333-3333-333333333333"
        with tempfile.TemporaryDirectory() as directory:
            sessions = Path(directory) / "sessions"
            sessions.mkdir()

            def rollout(session, source, row):
                write_session(sessions, f"rollout-{session}", {
                    "timestamp": STAMP,
                    "type": "session_meta",
                    "payload": {"id": session, "cwd": DIGEST["ROOT"], "source": source},
                }, row)

            rollout(parent, "cli", codex("function_call", call_id="read", arguments=f"cat {PLAN}"))
            rollout(child, {"subagent": {"thread_spawn": {
                "parent_thread_id": parent, "agent_path": "/root/step",
            }}}, codex("message", role="assistant", content=[]))
            rollout(incidental, "cli", codex("function_call_output", call_id="read", output=PLAN))

            def digest(*args):
                output = io.StringIO()
                main = DIGEST["main"]
                with (
                    patch.dict("os.environ", {"CODEX_HOME": directory}),
                    patch.dict(main.__globals__, {"project_dirs": lambda: []}),
                    patch.object(sys, "argv", ["session-digest", "--slug", SLUG, *args]),
                    contextlib.redirect_stdout(output),
                ):
                    main()
                return output.getvalue()

            selected = digest()
            self.assertIn(parent[:13], selected)
            self.assertIn(child[:13], selected)
            self.assertNotIn(incidental[:13], selected)
            self.assertIn(incidental[:13], digest("--broad"))


if __name__ == "__main__":
    unittest.main()
