"""Makes a 3D view of one simulated flight, as a web page you open in a browser. SIMULATOR ONLY.

What the page shows
-------------------
The organizers' 3D model of the course, the path the drone truly flew, and a small drone that
you can play back along that path. You can turn and zoom the view with the mouse.

How it works
------------
This script reads two files and writes one:

  - It reads one flight record that the referee wrote (generated_files/flight_records/flight_*.csv)
    and the 3D model of the course that flight flew in. The referee's summary names that model.
    Older flights, and flights in the organizers' drawing, use the organizers' model
    (course_model/dronehacks-course-v3.obj).
  - It copies the drawing code in viewer_page.html, puts the course and the flight inside it,
    and saves the result next to the flight record, ending in "_3d_view.html".

The page is a single file and needs no server. It does need an internet connection, because it
loads the 3D drawing library, three.js, from the internet.

How to run it
-------------
On the Mac, from the top folder of the repository. It needs only Python, with no extra
libraries. If you give no file name, it uses the newest flight record. On a Mac, it then opens
the page in your web browser.

    python3 simulator/flight_viewer/make_flight_viewer.py
    python3 simulator/flight_viewer/make_flight_viewer.py simulator/generated_files/flight_records/flight_XXX.csv
"""
import csv
import glob
import json
import os
import sys
import webbrowser

FOLDER_OF_THIS_SCRIPT = os.path.dirname(os.path.abspath(__file__))
SIMULATOR_FOLDER = os.path.dirname(FOLDER_OF_THIS_SCRIPT)
REPOSITORY_FOLDER = os.path.dirname(SIMULATOR_FOLDER)
# Inside the simulator's Linux box, the repository is at this path (see run_test_flight.sh).
REPOSITORY_FOLDER_IN_THE_BOX = "/repository"
COURSE_MODEL_FILE = os.path.join(SIMULATOR_FOLDER, "course_model", "dronehacks-course-v3.obj")
FLIGHT_RECORDS_FOLDER = os.path.join(SIMULATOR_FOLDER, "generated_files", "flight_records")
PAGE_TEMPLATE_FILE = os.path.join(FOLDER_OF_THIS_SCRIPT, "viewer_page.html")

# viewer_page.html has this word in the place where the data goes.
DATA_PLACEHOLDER = "FLIGHT_AND_COURSE_DATA_GOES_HERE"

# Our own choice. Positions are rounded to the nearest millimetre, which keeps the page small.
# A millimetre is far finer than anything we can see on the screen.
DECIMAL_PLACES_FOR_METRES = 3

# The kinds of course part. The page draws each kind differently. These names match the ones
# in course_simulator/course_model.py, which starts from the organizers' part names.
NET_PART_NAME = "Net"
FLOOR_PART_NAME = "Floor"
FLOOR_MARKING_PART_NAMES = ("Guideline", "Landing", "Launch", "Approach ring")


class CoursePart:
    """One named part of the course, such as "Boulder 1", in the form the page draws.

    The model file numbers its corner points once for the whole file. The page wants each part
    to have its own short list, so this class renumbers the corners of its own part.
    """

    def __init__(self, name, colour_red_green_blue):
        self.name = name
        self.colour_red_green_blue = colour_red_green_blue
        self.corner_points_metres = []  # x, y, z, x, y, z, ... one after another.
        self.triangles = []  # Three corner numbers per triangle, counted within this part.
        self._part_corner_number_for_file_corner_number = {}

    def add_face(self, file_corner_numbers, all_corner_points_metres):
        part_corner_numbers = []
        for file_corner_number in file_corner_numbers:
            part_corner_numbers.append(
                self._part_corner_number(file_corner_number, all_corner_points_metres)
            )
        # A face with more than three corners is split into triangles that share its first corner.
        for index in range(1, len(part_corner_numbers) - 1):
            self.triangles.append(part_corner_numbers[0])
            self.triangles.append(part_corner_numbers[index])
            self.triangles.append(part_corner_numbers[index + 1])

    def _part_corner_number(self, file_corner_number, all_corner_points_metres):
        if file_corner_number not in self._part_corner_number_for_file_corner_number:
            new_number = len(self.corner_points_metres) // 3
            self._part_corner_number_for_file_corner_number[file_corner_number] = new_number
            for value in all_corner_points_metres[file_corner_number]:
                self.corner_points_metres.append(round(value, DECIMAL_PLACES_FOR_METRES))
        return self._part_corner_number_for_file_corner_number[file_corner_number]

    def kind(self):
        if self.name.startswith(NET_PART_NAME):
            return "net"
        if self.name.startswith(FLOOR_PART_NAME):
            return "floor"
        if self.name.startswith(FLOOR_MARKING_PART_NAMES):
            return "floor marking"
        return "obstacle"

    def for_the_page(self):
        return {
            "name": self.name,
            "kind": self.kind(),
            "colour": self.colour_red_green_blue,
            "corner_points_metres": self.corner_points_metres,
            "triangles": self.triangles,
        }


def colour_from_material_name(material_name):
    """Returns the red, green and blue amounts, from 0 to 1, that a material name describes.

    Onshape, the program that made the course model, names each material after its colour,
    for example "0.803922_0.372549_0.235294_0.000000_0.000000". The first three numbers are the
    red, green and blue amounts. We don't need the last two.
    """
    numbers = material_name.split("_")
    return [float(numbers[0]), float(numbers[1]), float(numbers[2])]


def read_course_parts(path_to_model_file):
    """Reads the course model and returns its parts as a list of CourseParts.

    The file format is plain text. A line starting with "v" is a corner point, "g" starts a new
    named part, "usemtl" chooses the colour for what follows, and "f" is a face that joins
    corner points. Other lines don't matter for drawing.
    """
    all_corner_points_metres = []
    parts_by_name = {}
    current_part_name = None
    current_colour = [0.5, 0.5, 0.5]  # A grey for anything drawn before the file names a colour.

    with open(path_to_model_file, encoding="utf-8") as model_file:
        for line in model_file:
            words = line.split()
            if not words:
                continue
            if words[0] == "v":
                all_corner_points_metres.append([float(word) for word in words[1:4]])
            elif words[0] == "g":
                current_part_name = line[2:].strip()
            elif words[0] == "usemtl":
                current_colour = colour_from_material_name(words[1])
            elif words[0] == "f" and current_part_name is not None:
                if current_part_name not in parts_by_name:
                    parts_by_name[current_part_name] = CoursePart(current_part_name, current_colour)
                # A corner can be written "12//3". Only the first number matters, and the file
                # counts from 1, while Python counts from 0.
                file_corner_numbers = [int(word.split("/")[0]) - 1 for word in words[1:]]
                parts_by_name[current_part_name].add_face(file_corner_numbers, all_corner_points_metres)

    return list(parts_by_name.values())


def yes_no_or_unknown(text):
    """The flight record writes True, False, or nothing when the state was not known yet."""
    if text == "True":
        return True
    if text == "False":
        return False
    return None


def read_flight_record(path_to_record):
    """Returns the record's samples, one per row, in time order."""
    samples = []
    with open(path_to_record, newline="", encoding="utf-8") as record_file:
        reader = csv.DictReader(record_file)
        if "heading_degrees" not in reader.fieldnames:
            raise ValueError(
                f"{os.path.basename(path_to_record)} was recorded before the simulator saved "
                "which way the drone faces, so the drone can't be drawn. Fly a new test flight "
                "and use its record instead."
            )
        for row in reader:
            samples.append({
                "seconds": float(row["seconds"]),
                "course_x_metres": float(row["course_x_metres"]),
                "course_y_metres": float(row["course_y_metres"]),
                "height_metres": float(row["height_metres"]),
                "heading_degrees": float(row["heading_degrees"]),
                "nearest_obstacle_metres": float(row["nearest_obstacle_metres"]),
                "armed": yes_no_or_unknown(row["armed"]),
                "landed": yes_no_or_unknown(row["landed"]),
                "automatic_flight": yes_no_or_unknown(row["automatic_flight"]),
            })
    if not samples:
        raise ValueError(f"{os.path.basename(path_to_record)} has no samples in it, so there is nothing to show.")
    return samples


def read_summary(path_to_record):
    """Returns the referee's summary of this flight, or None if there isn't one."""
    path_to_summary = path_to_record.replace(".csv", "_summary.json")
    if not os.path.exists(path_to_summary):
        return None
    with open(path_to_summary, encoding="utf-8") as summary_file:
        return json.load(summary_file)


def find_newest_record():
    records = sorted(glob.glob(os.path.join(FLIGHT_RECORDS_FOLDER, "flight_*[0-9].csv")))
    if not records:
        raise FileNotFoundError(f"There are no flight records in {FLIGHT_RECORDS_FOLDER}. Run a test flight first.")
    return records[-1]


def write_page(page_data, path_to_page):
    with open(PAGE_TEMPLATE_FILE, encoding="utf-8") as template_file:
        template = template_file.read()
    if template.count(DATA_PLACEHOLDER) != 1:
        raise ValueError(
            f"viewer_page.html must contain the word {DATA_PLACEHOLDER} exactly once, to mark "
            "where the data goes. Check that the file hasn't been changed by mistake."
        )

    data_as_text = json.dumps(page_data, separators=(",", ":"))
    # The data sits inside a <script> block on the page, and the text "</" could end that block
    # early. Writing it as "<\/" means the same thing to the page's code, but not to the browser.
    data_as_text = data_as_text.replace("</", "<\\/")

    with open(path_to_page, "w", encoding="utf-8") as page_file:
        page_file.write(template.replace(DATA_PLACEHOLDER, data_as_text))


def course_model_used_by(summary):
    """The 3D model of the course that a flight flew in, as a path on this computer."""
    if summary is None or "course_model_file" not in summary:
        return COURSE_MODEL_FILE
    path_in_the_box = summary["course_model_file"]
    path_inside_repository = os.path.relpath(path_in_the_box, REPOSITORY_FOLDER_IN_THE_BOX)
    return os.path.join(REPOSITORY_FOLDER, path_inside_repository)


def main():
    path_to_record = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else find_newest_record()
    summary = read_summary(path_to_record)
    page_data = {
        "flight_record_name": os.path.basename(path_to_record),
        "samples": read_flight_record(path_to_record),
        "summary": summary,
        "course_parts": [part.for_the_page() for part in read_course_parts(course_model_used_by(summary))],
    }

    path_to_page = path_to_record.replace(".csv", "_3d_view.html")
    write_page(page_data, path_to_page)
    print(f"Saved the 3D view to {path_to_page}")

    # Inside the simulator's Linux box there is no browser, so only open it on a Mac.
    if sys.platform == "darwin":
        webbrowser.open("file://" + path_to_page)


if __name__ == "__main__":
    main()
