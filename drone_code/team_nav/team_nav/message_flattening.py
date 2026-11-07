"""Turns any ROS message into one flat row of named values, so it can be written as a CSV row.

REAL DRONE CODE. The recorder (flight_recorder.py) uses it for every topic it records, whatever
the message type, so it never needs to know a message's fields in advance.

How a message becomes a row
---------------------------
A ROS message is a set of named fields, and a field may itself be a message. For example an
odometry message has pose.pose.position.x. We walk through the fields and give each plain value
its own column, named by its path: "pose.pose.position.x".

Lists need care, because the first row of a CSV file decides its columns for good:
  - a list whose length is fixed by the message definition, and is short (up to 9 values, like
    a quaternion's 4 or a 3 x 3 matrix's 9), gets one column per value: "q[0]", "q[1]" ...;
  - any other list (a variable-length one, or a long one such as a 6 x 6 covariance matrix)
    goes into a single column as JSON text, for example "[0.1, 0.2, 0.3]".
So every message of one type always gives the same columns.

Every ROS message class has a method get_fields_and_field_types(), which lists its fields and
their types as text, such as "float[4]" (fixed length 4), "sequence<float>" (any length) or
"geometry_msgs/Pose" (another message). That is all this file relies on.

What it does not do
-------------------
It does not subscribe, choose topics or write files. It does not round numbers: the CSV keeps
every digit.
"""
import json

# Our own choice: fixed-length lists up to this long get one column per value.
LONGEST_LIST_WITH_ONE_COLUMN_PER_VALUE = 9


def flatten_message(message):
    """Returns {column name: value} for one message. Values are numbers, booleans or text."""
    row = {}
    _add_fields(message, "", row)
    return row


def message_to_plain_data(message):
    """The whole message as plain dictionaries, lists and numbers, ready for JSON."""
    if _is_message(message):
        plain = {}
        for field_name in message.get_fields_and_field_types():
            plain[field_name] = message_to_plain_data(getattr(message, field_name))
        return plain
    if isinstance(message, (bytes, bytearray)):
        return list(message)
    if _is_list_like(message) and hasattr(message, "tolist"):
        # A numpy array or a Python array: tolist() gives plain Python numbers, quickly.
        return message.tolist()
    if _is_list_like(message):
        plain_list = []
        for item in message:
            plain_list.append(message_to_plain_data(item))
        return plain_list
    return _as_plain_value(message)


def _add_fields(message, name_prefix, row):
    for field_name, field_type in message.get_fields_and_field_types().items():
        column_name = name_prefix + field_name
        value = getattr(message, field_name)
        if _is_message(value):
            _add_fields(value, column_name + ".", row)
        elif _is_list_like(value) or isinstance(value, (bytes, bytearray)):
            _add_list(column_name, field_type, value, row)
        else:
            row[column_name] = _as_plain_value(value)


def _add_list(column_name, field_type, value, row):
    fixed_length = _fixed_list_length(field_type)
    items = message_to_plain_data(value)
    short_fixed_list_of_plain_values = (
        fixed_length is not None
        and fixed_length <= LONGEST_LIST_WITH_ONE_COLUMN_PER_VALUE
        and not any(isinstance(item, (dict, list)) for item in items)
    )
    if not short_fixed_list_of_plain_values:
        row[column_name] = json.dumps(items)
        return
    for position in range(fixed_length):
        row[f"{column_name}[{position}]"] = items[position] if position < len(items) else None


def _fixed_list_length(field_type):
    """4 for "float[4]"; None for "sequence<float>", "sequence<float, 8>" and non-lists.
    Only "name[N]" is fixed: a "sequence" may be shorter than its limit."""
    if not field_type.endswith("]") or "[" not in field_type:
        return None
    length_text = field_type[field_type.rindex("[") + 1:-1]
    return int(length_text) if length_text.isdigit() else None


def _is_message(value):
    return hasattr(value, "get_fields_and_field_types")


def _is_list_like(value):
    """True for lists, tuples, Python arrays and numpy arrays, which is how ROS gives list
    fields to Python. Text is not a list here."""
    if isinstance(value, (str, bytes, bytearray, dict)):
        return False
    # A single numpy number also has tolist(), but it has no length.
    return isinstance(value, (list, tuple)) or (hasattr(value, "tolist") and hasattr(value, "__len__"))


def _as_plain_value(value):
    """Numpy numbers (which ROS uses inside fixed-length lists) become plain Python numbers."""
    if hasattr(value, "item") and not isinstance(value, (str, bytes)):
        return value.item()
    return value
