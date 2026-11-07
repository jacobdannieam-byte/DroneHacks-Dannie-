"""Reads the one setting that says whether this is a competition run or a practice run.

REAL DRONE CODE. Every program in this package that flies or records reads its mode here, so
they can never disagree.

Why there is a switch at all
----------------------------
Before the competition run we fly the course many times to collect data and tune the program.
That is practice mode. Section 11 of docs/ARCHITECTURE.md describes it. The flying itself is
the same in both modes, so practice data is representative. The only differences are:
  - a practice run can be told to stop after a chosen stage and land, to practise one stage at
    a time;
  - the recorder may save more data in practice mode, such as camera pictures, because the
    processor has less to do when nothing is at stake.

The setting
-----------
The mode comes from the environment variable TEAM_NAV_RUN_MODE, read once when a program
starts. It is "competition" or "practice". If it is missing or empty, the mode is competition,
so forgetting the switch can never leave practice mode on during the scored run.

TEAM_NAV_STOP_AFTER_STAGE names the stage after which a practice run lands. Each flight
program says which stage names it understands. It is refused in competition mode.

This file does not decide anything else about the flight. It only reads and checks the setting.
"""
import os
from dataclasses import dataclass

# Our own choices: the names of the two settings and of the two modes.
RUN_MODE_SETTING = "TEAM_NAV_RUN_MODE"
STOP_AFTER_STAGE_SETTING = "TEAM_NAV_STOP_AFTER_STAGE"
COMPETITION = "competition"
PRACTICE = "practice"


@dataclass(frozen=True)
class RunMode:
    name: str                 # COMPETITION or PRACTICE.
    stop_after_stage: str     # A stage name, or "" to fly the whole program.

    @property
    def is_practice(self):
        return self.name == PRACTICE

    def describe(self):
        """One sentence for the screen and the log."""
        if self.name == COMPETITION:
            return "COMPETITION MODE: the whole program flies, exactly as in the scored run."
        if self.stop_after_stage:
            return f"PRACTICE MODE: the run lands after the stage '{self.stop_after_stage}'."
        return "PRACTICE MODE: the whole program flies, and the recorder saves extra data."


def read_run_mode(environment=None):
    """Returns the RunMode from the environment. Raises ValueError, with a sentence saying how to
    fix it, if the setting has a value we don't recognise."""
    if environment is None:
        environment = os.environ
    mode_name = environment.get(RUN_MODE_SETTING, "").strip().lower()
    stop_after_stage = environment.get(STOP_AFTER_STAGE_SETTING, "").strip()

    if mode_name == "":
        mode_name = COMPETITION
    if mode_name not in (COMPETITION, PRACTICE):
        raise ValueError(
            f"{RUN_MODE_SETTING} is '{environment.get(RUN_MODE_SETTING)}', which is not a mode. "
            f"Set it to '{PRACTICE}' or '{COMPETITION}', or remove it for competition mode."
        )
    if mode_name == COMPETITION and stop_after_stage:
        raise ValueError(
            f"{STOP_AFTER_STAGE_SETTING} is set to '{stop_after_stage}', but this is competition mode, "
            f"where the whole program must fly. Remove {STOP_AFTER_STAGE_SETTING}, or set "
            f"{RUN_MODE_SETTING}={PRACTICE}."
        )
    return RunMode(mode_name, stop_after_stage)
