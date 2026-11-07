# How we write code in this repository

This guide applies to every file we write here, whether a person or a coding
agent writes it. It is reloaded into the coding agent after every context
compaction, so it must stay short and true.

## Who we are writing for

The reader is an electrical engineer who has never worked with drones. They are
smart, they can read Python, and they do not know what "Offboard", "EKF2" or
"NED" mean. They want to understand the code, not just run it.

If they have to stop and ask "what does this mean?" or "why is this here?", then
the code has failed, even if it works.

## What makes code readable

Code is hard to read for a few specific reasons, and each rule below answers one.

### 1. The reader does not know where to start

Every folder that holds code has a `README.md`. It says what the folder is for,
which file to read first, and in what order to read the rest. The folders inside
one small package can share a single README.

Every file opens with a plain-English explanation of three things: what this file
does, how it fits with the other files, and what it deliberately does not do.

### 2. The reader does not know the words

Name things after what they mean, in full words.

| Hard to read | Easy to read |
|---|---|
| `alt` | `height_above_floor_metres` |
| `vz` | `upward_speed_metres_per_second` |
| `cb` | `on_new_position_report` |
| `t0`, `dt` | `start_time_seconds`, `seconds_since_last_step` |
| `odom` | `visual_position_estimate` |

Put the unit in the name of any physical quantity: `_metres`, `_seconds`,
`_kilograms`, `_newtons`, `_degrees`. In electronics we would never write down a
voltage without saying volts.

The first time a file uses a drone term, explain it in a comment, in one plain
sentence. Do this even if another file already explains it, because the reader
may open this file first.

```python
# "Arming" means giving the motors permission to spin. A disarmed drone cannot
# fly, whatever the software asks for.
```

External names, such as a PX4 setting called `MPC_Z_VEL_MAX_DN`, cannot be
renamed. Say what they mean next to them.

### 3. The reader cannot see why something was done

Comments explain why, not what. The code already says what it does.

```python
# Bad: this only repeats the code.
speed = 0.35  # set speed to 0.35

# Good: this says why the number is what it is.
# The land detector only counts the drone as landed if it is moving down slower
# than 0.25 m/s, so we must arrive below that. We start a little faster than
# that and slow down near the floor.
```

When code looks strange, say so and say why. A strange line with an explanation
is fine. A strange line without one looks like a bug.

### 4. The reader cannot tell where a number came from

There are no unexplained numbers in the logic. Each one is a named constant at
the top of the file, with its unit and where it came from:

```python
# Measured on drone 16 with a kitchen scale on 2026-11-07.
DRONE_MASS_KILOGRAMS = 1.47

# Estimated. We have no measurement yet. It comes from the setting that says the
# motors hover at half power, so full power must be about twice the weight.
FULL_THRUST_PER_MOTOR_NEWTONS = 7.2
```

Always label a value as one of: **measured**, **estimated**, **from PX4 source
code**, **from the rulebook**, **from the organizers' code**, or **our own
choice**. A reader must be able to tell a fact from a guess.

### 5. The reader has to hold too much in their head

Each file has one job. If describing a file needs the word "and", it is probably
two files.

Each function does one thing, fits on one screen, and reads from top to bottom
like a list of steps. Prefer a short sequence of well-named calls over one long
block.

```python
def run_landing():
    descend_until_close_to_floor()
    slow_down_for_touchdown()
    push_gently_into_floor_until_landed()
    confirm_motors_have_stopped()
```

Keep nesting shallow. Return early rather than stacking `if` inside `if`.

### 6. The code is too clever

Write the obvious version, even if it is longer. Do not use a one-line
comprehension that needs a second reading, a metaclass, a decorator that hides
behaviour, or a global variable. Use a normal loop.

Speed matters only where we have measured a problem. If a section really must be
fast and therefore hard to read, say so in a comment and explain what it does in
plain words.

### 7. The reader cannot tell what is real and what is pretend

Simulator code and real flight code are kept in separate folders and never mix.
Anything that only exists in the simulator says so in its name, in its file
header, and in the ROS topic name. A file written for the simulator must never
be able to run on the drone by accident.

Code we copied from the organizers stays unchanged, in its own folder, with a
note saying where it came from and which version it is. We do not reformat it,
because then we could not compare it with the original.

### 8. Errors do not say what to do

An error message is a full sentence that says what went wrong and what to do
next.

```python
# Bad
raise RuntimeError("no odom")

# Good
raise RuntimeError(
    "No position report has arrived for 2 seconds, so the drone does not know "
    "where it is. Check that the camera tracking program is running."
)
```

### 9. The reader cannot trust the documentation

If the code and a comment disagree, the comment is a bug. When you change the
code, change the comment in the same edit. Delete comments and documents that are
no longer true. Say "we have not tested this" when that is the truth.

## Checklist before finishing a file

- Does the file start by saying what it does and how it fits in?
- Could someone with no drone experience follow it from top to bottom?
- Does every physical quantity have its unit in its name?
- Does every number have a name and a stated source?
- Does every strange line say why it is there?
- Is any function too long to fit on one screen?
- Is it obvious whether this code is for the simulator or for the real drone?

## The same rules apply when talking

When explaining anything to the team, use full, conversational sentences. Define
drone terms or avoid them. Say what something means and why it matters before
giving detail. Never leave a statement ambiguous. Do not exaggerate risks.
