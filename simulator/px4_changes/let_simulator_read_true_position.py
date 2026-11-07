"""Lets the simulator read the drone's true position out of PX4.

SIMULATOR ONLY. It runs once, while the simulator image is being built. The real drone never
gets this change.

What it does
------------
PX4 has a list, in a file called dds_topics.yaml, of which internal messages it
shares with ROS. We add two messages to that list: the drone's TRUE position and
TRUE attitude (which way it is tilted and pointing). They exist only because PX4
is simulating the physics, so a real drone has no such thing.

Why the simulator needs them
----------------------------
PX4's built-in physics knows where the drone truly is, but it does not know about
walls. Our world programs need the true position so they can work out:

  - what the bottom laser would read, by looking straight down in the course model
  - what the camera tracking would report, by adding errors to the true position
  - whether the drone has touched a wall

This changes no flight logic. PX4 only tells us more; it decides nothing differently.
"""
import sys

# "type" names the message's format, which is the same as the ordinary versions of
# these messages. PX4 works out the internal message name from the end of "topic".
LINES_TO_ADD = """\
  - topic: /fmu/out/vehicle_local_position_groundtruth
    type: px4_msgs::msg::VehicleLocalPosition

  - topic: /fmu/out/vehicle_attitude_groundtruth
    type: px4_msgs::msg::VehicleAttitude

"""

MARKER_WHERE_THE_INPUT_LIST_STARTS = "subscriptions:\n"


def main():
    path_to_topic_list = sys.argv[1]

    with open(path_to_topic_list, encoding="utf-8") as topic_file:
        original_text = topic_file.read()

    if MARKER_WHERE_THE_INPUT_LIST_STARTS not in original_text:
        raise RuntimeError(
            f"I could not find the line 'subscriptions:' in {path_to_topic_list}. "
            "PX4 must have changed this file's layout, so this script needs updating."
        )

    # The list of messages PX4 shares with ROS ends where the list of messages it
    # receives begins, so the new lines go just before that point.
    changed_text = original_text.replace(
        MARKER_WHERE_THE_INPUT_LIST_STARTS,
        LINES_TO_ADD + MARKER_WHERE_THE_INPUT_LIST_STARTS,
        1,
    )

    with open(path_to_topic_list, "w", encoding="utf-8") as topic_file:
        topic_file.write(changed_text)

    print("Added the true position and attitude messages to PX4's topic list.")


if __name__ == "__main__":
    main()
