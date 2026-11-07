"""Keep teammate referee failures visible in the integrated launch exit status."""

import json
from pathlib import Path
import sys


def main(folder):
    files = sorted(Path(folder).glob("*_summary.json"))
    if len(files) != 1:
        print(f"Expected one referee summary, found {len(files)}", file=sys.stderr)
        return 1
    summary = json.loads(files[0].read_text())
    hazards = (summary.get("touches") != 0 or bool(summary.get("height_estimate_went_wrong"))
               or bool(summary.get("stayed_above_a_boulder_too_long")) or bool(summary.get("went_above_walls"))
               or bool(summary.get("left_the_course")))
    points = summary.get("checkpoint_seconds") or {}
    full_course = (not hazards and len(points) == 4 and all(value is not None for value in points.values())
                   and summary.get("landed_on_pad") is True and summary.get("motors_off_seconds") is not None)
    print(json.dumps({"independent_full_course_checks_passed": full_course, "referee": summary}, indent=2))
    return int(hazards)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]))
