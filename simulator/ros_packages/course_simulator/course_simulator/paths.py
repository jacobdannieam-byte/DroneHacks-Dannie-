"""Where the simulator's files are, inside the Linux box. SIMULATOR ONLY.

The repository is always shared into the box at /repository (see run_test_flight.sh).
world.py decides which course the simulator flies in, using these paths.
"""
# The organizers' own 3D model of the course, and the map file we made from it.
COURSE_MODEL_FILE = "/repository/simulator/course_model/dronehacks-course-v3.obj"
ORGANIZERS_DRAWING_MAP_FILE = "/repository/drone_code/team_nav/course_maps/organizers_drawing.json"

# The checkpoints the rulebook asks for, in order. The referee judges every flight by these.
RULEBOOK_MISSION_FILE = "/repository/drone_code/team_nav/missions/dronehacks_2026.json"

FOLDER_FOR_SAVED_TABLES = "/repository/simulator/generated_files/course_tables"
FOLDER_FOR_GENERATED_WORLDS = "/repository/simulator/generated_files/worlds"
FOLDER_FOR_FLIGHT_RECORDS = "/repository/simulator/generated_files/flight_records"
