"""Turns a course map into a Gazebo world file. TRIAL ONLY, SIMULATOR ONLY.

What this file does
-------------------
Gazebo is a robot simulator. It reads the world it simulates from a text file in a format
called SDF ("Simulation Description Format"), which is XML. This file takes a course map
(the same map files our route planner reads, see drone_code/team_nav/team_nav/course_map.py)
and writes the SDF text for that course:

  - the floor;
  - every wall, as solid boxes, with its openings (such as the window) left empty;
  - the oval tunnel, as a ring of thin flat strips running along its length;
  - every low obstacle, such as a boulder, as an upright cylinder;
  - the safety net around the course, and its roof, as thin solid sheets;
  - the lights, and the list of Gazebo add-ons ("systems") that the world needs.

Every shape is both seen by Gazebo's cameras ("visual") and solid to Gazebo's physics
("collision"), so a simulated drone that flies into a wall really hits it.

How it fits with the other files
--------------------------------
make_gazebo_world.py is the command you run. It loads a map, calls world_sdf() from this
file, adds the drone, and saves the result. measure_rendering_speed.py also calls
world_sdf(), and adds a fixed camera instead of a drone.

What it does not do
-------------------
It does not read the organizers' 3D model or the phone scan directly, only map files. So
the world has exactly the simple shapes that the map has: straight walls, one kind of
tube, and cylinders. It also knows nothing about checkpoints, the launch mark or the
landing pad, because those are lines and marks on the floor, which nothing can hit.

Positions
---------
Gazebo's x, y and z are the same as the course map's: x and y along the floor with (0, 0)
at the centre of the course, and z up from the floor, all in metres.
"""
import math

# The name Gazebo uses for the world. PX4 needs to know it, so we keep it the same
# whichever map the world was made from. Our own choice.
WORLD_NAME = "dronehacks_course"

# From the organizers' 3D model (dronehacks-course-v3.obj): the net's four sides reach from
# the floor to 3.048 m (10 ft), and a flat net roof covers the whole course at that height.
# The map files do not record this, so it is a constant here.
NET_HEIGHT_METRES = 3.048

# Our own choice. The real net is a thin mesh, but a thin solid is a problem in Gazebo: the
# physics checks for contact once every 4 ms, so something moving at 5 m/s moves 2 cm between
# checks and can pass through a 2 cm sheet. In one test, a drone that crashed at about that
# speed did end up outside a 2 cm net. The extra thickness lies outside the course edge, so
# the course keeps its full size.
NET_THICKNESS_METRES = 0.10

# Our own choice. How many flat strips make up the oval tunnel. With 48 strips on a tunnel
# about 1.5 m wide, each strip is about 10 cm wide, and a flat strip strays from the true
# curved surface by at most about 2 mm.
TUNNEL_STRIP_COUNT = 48

# Our own choice. How far the floor reaches beyond the net on each side, so that a camera
# looking out of the course near a corner still sees floor rather than empty space.
FLOOR_MARGIN_METRES = 5.0

# From PX4's own Gazebo worlds (Tools/simulation/gz/worlds/default.sdf in PX4 v1.16.2).
# Gazebo works out the physics in steps of 4 milliseconds, 250 times per simulated second.
PHYSICS_STEP_SECONDS = 0.004

# From PX4's default.sdf. PX4's simulated GPS and compass need the world to have a place on
# Earth and a magnetic field. These numbers are for Zurich, where PX4 is developed. They do
# not matter indoors, but PX4 will not start its Gazebo link without them.
LATITUDE_DEGREES = 47.397971057728974
LONGITUDE_DEGREES = 8.546163739800146
MAGNETIC_FIELD_TESLA = (6e-06, 2.3e-05, -4.2e-05)

# Our own choice of colours, as red, green, blue, opacity, each from 0 to 1. They only change
# how pictures look, not depth.
FLOOR_COLOUR = (0.55, 0.55, 0.55, 1.0)
WALL_COLOUR = (0.85, 0.80, 0.70, 1.0)
TUNNEL_COLOUR = (0.30, 0.45, 0.75, 1.0)
LOW_OBSTACLE_COLOUR = (0.45, 0.35, 0.25, 1.0)
NET_COLOUR = (0.15, 0.15, 0.15, 1.0)

# Our own choice: the name of the reference point that every course object is placed from.
COURSE_FRAME_NAME = "course"


# ---- The whole world -------------------------------------------------------------------------

def world_sdf(course_map, extra_sdf="", course_origin_in_world_metres=(0.0, 0.0, 0.0)):
    """Returns the SDF text for a whole Gazebo world built from this course map.
    extra_sdf is placed inside the world as it is, for example a drone.

    course_origin_in_world_metres says where the course's (0, 0, 0), the centre of the course on
    the floor, sits in Gazebo's world, as (x, y, z). Every course object is placed relative to a
    frame (a named reference point) called "course" at that spot. Moving it moves the whole
    course at once, floor included, without changing any shape."""
    origin_x, origin_y, origin_z = course_origin_in_world_metres
    return f"""<?xml version="1.0"?>
<!-- Made by simulator/gazebo_trial/make_gazebo_world.py from the course map "{_escape(course_map.name)}".
     Do not edit this file by hand: change the map file and make it again. -->
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <frame name="{COURSE_FRAME_NAME}"><pose>{origin_x:.4f} {origin_y:.4f} {origin_z:.4f} 0 0 0</pose></frame>
{_physics_and_systems_sdf()}
{_lights_sdf()}
{_floor_sdf(course_map)}
{_safety_net_sdf(course_map)}
{_all_walls_sdf(course_map)}
{_all_tubes_sdf(course_map)}
{_all_low_obstacles_sdf(course_map)}
{extra_sdf}
  </world>
</sdf>
"""


def _physics_and_systems_sdf():
    """The physics settings, and the list of Gazebo systems the world loads.

    Each system is an add-on that gives the world one ability. Most come from PX4's own
    server.config, because PX4's simulated drone needs them: the motion sensor (Imu), the
    air pressure sensor that PX4 uses as an altimeter (AirPressure), the compass
    (Magnetometer), the GPS (NavSat), and Sensors, which draws camera pictures.
    """
    magnetic_x, magnetic_y, magnetic_z = MAGNETIC_FIELD_TESLA
    return f"""    <physics type="ode">
      <max_step_size>{PHYSICS_STEP_SECONDS}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>{1.0 / PHYSICS_STEP_SECONDS:.0f}</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>{magnetic_x} {magnetic_y} {magnetic_z}</magnetic_field>
    <atmosphere type="adiabatic"/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{LATITUDE_DEGREES}</latitude_deg>
      <longitude_deg>{LONGITUDE_DEGREES}</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <!-- "ogre2" is the drawing program Gazebo uses for camera pictures. -->
      <render_engine>ogre2</render_engine>
    </plugin>"""


def _lights_sdf():
    """Soft light from everywhere, plus one light from straight above.

    Shadows are turned off. Our own choice: there is no graphics card in the box, so the
    processor draws every picture, and shadows would make each picture slower to draw.
    A depth picture does not need them."""
    return """    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
    <light name="light_from_above" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.2 0.3 -1</direction>
    </light>"""


# ---- The floor and the net -------------------------------------------------------------------

def _floor_sdf(course_map):
    width_metres = course_map.highest_corner_metres[0] - course_map.lowest_corner_metres[0]
    length_metres = course_map.highest_corner_metres[1] - course_map.lowest_corner_metres[1]
    floor_width_metres = width_metres + 2.0 * FLOOR_MARGIN_METRES
    floor_length_metres = length_metres + 2.0 * FLOOR_MARGIN_METRES
    # A plane is an endless flat surface. Its size only matters for how much of it is drawn.
    plane = f"<plane><normal>0 0 1</normal><size>{floor_width_metres:.3f} {floor_length_metres:.3f}</size></plane>"
    return _static_model_sdf("floor", [_shape_sdf("floor", plane, (0.0, 0.0, 0.0), 0.0, 0.0, FLOOR_COLOUR)])


def _safety_net_sdf(course_map):
    """The four sides of the net, standing on the course edges, and its flat roof. The inside
    face of each side lies exactly on the course edge, so the course keeps its full size."""
    low_x, low_y = course_map.lowest_corner_metres
    high_x, high_y = course_map.highest_corner_metres
    half_thickness_metres = NET_THICKNESS_METRES / 2.0
    middle_x = (low_x + high_x) / 2.0
    middle_y = (low_y + high_y) / 2.0
    outside_width_metres = high_x - low_x + 2.0 * NET_THICKNESS_METRES
    outside_length_metres = high_y - low_y + 2.0 * NET_THICKNESS_METRES
    middle_height_metres = NET_HEIGHT_METRES / 2.0

    # Each side: (name, centre x, centre y, size along x, size along y, size along z).
    sheets = [
        ("net_side_west", low_x - half_thickness_metres, middle_y, NET_THICKNESS_METRES, outside_length_metres, NET_HEIGHT_METRES),
        ("net_side_east", high_x + half_thickness_metres, middle_y, NET_THICKNESS_METRES, outside_length_metres, NET_HEIGHT_METRES),
        ("net_side_south", middle_x, low_y - half_thickness_metres, outside_width_metres, NET_THICKNESS_METRES, NET_HEIGHT_METRES),
        ("net_side_north", middle_x, high_y + half_thickness_metres, outside_width_metres, NET_THICKNESS_METRES, NET_HEIGHT_METRES),
    ]
    shapes = []
    for name, centre_x, centre_y, size_x, size_y, size_z in sheets:
        shapes.append(_box_sdf(name, (centre_x, centre_y, middle_height_metres), 0.0, 0.0, (size_x, size_y, size_z), NET_COLOUR))
    roof_centre = (middle_x, middle_y, NET_HEIGHT_METRES + half_thickness_metres)
    roof_size = (outside_width_metres, outside_length_metres, NET_THICKNESS_METRES)
    shapes.append(_box_sdf("net_roof", roof_centre, 0.0, 0.0, roof_size, NET_COLOUR))
    return _static_model_sdf("safety_net", shapes)


# ---- Walls -----------------------------------------------------------------------------------

def _all_walls_sdf(course_map):
    models = []
    for wall in course_map.walls:
        models.append(_wall_sdf(wall))
    return "\n".join(models)


def _wall_sdf(wall):
    """One wall, as one box per solid rectangle that is left after cutting out its openings."""
    start_x, start_y = wall.start_metres
    along_x, along_y = wall.direction()
    # The wall's heading: the angle from the x axis to the wall, turning towards the y axis.
    heading_radians = math.atan2(along_y, along_x)

    shapes = []
    for number, piece in enumerate(solid_pieces_of_wall(wall)):
        start_along_metres, end_along_metres, bottom_metres, top_metres = piece
        middle_along_metres = (start_along_metres + end_along_metres) / 2.0
        centre = (
            start_x + along_x * middle_along_metres,
            start_y + along_y * middle_along_metres,
            (bottom_metres + top_metres) / 2.0,
        )
        size = (end_along_metres - start_along_metres, wall.thickness_metres, top_metres - bottom_metres)
        shapes.append(_box_sdf(f"piece_{number}", centre, heading_radians, 0.0, size, WALL_COLOUR))
    return _static_model_sdf(wall.name, shapes)


def solid_pieces_of_wall(wall):
    """Splits a wall, seen face on, into solid rectangles around its openings.

    Returns a list of (start along the wall, end along the wall, bottom height, top height),
    all in metres. A wall with no openings is one rectangle. A wall with one window becomes
    four: the full-height part on each side, and the parts below and above the window.

    How: we cut the wall into vertical slices at every opening's left and right edges.
    Inside one slice, every opening either covers the whole slice width or none of it, so
    each slice is a simple column with some height ranges removed.
    """
    wall_length_metres = wall.length_metres()
    slice_edges_metres = {0.0, wall_length_metres}
    for opening in wall.openings:
        slice_edges_metres.add(min(max(opening.start_along_wall_metres, 0.0), wall_length_metres))
        slice_edges_metres.add(min(max(opening.end_along_wall_metres, 0.0), wall_length_metres))
    slice_edges_metres = sorted(slice_edges_metres)

    pieces = []
    for slice_start_metres, slice_end_metres in zip(slice_edges_metres[:-1], slice_edges_metres[1:]):
        solid_heights = [(wall.bottom_height_metres, wall.top_height_metres)]
        for opening in wall.openings:
            covers_this_slice = (
                opening.start_along_wall_metres <= slice_start_metres
                and opening.end_along_wall_metres >= slice_end_metres
            )
            if covers_this_slice:
                solid_heights = _remove_height_range(solid_heights, opening.bottom_height_metres, opening.top_height_metres)
        for bottom_metres, top_metres in solid_heights:
            pieces.append((slice_start_metres, slice_end_metres, bottom_metres, top_metres))
    return pieces


def _remove_height_range(height_ranges, removed_bottom_metres, removed_top_metres):
    """Removes one height range from a list of (bottom, top) height ranges."""
    remaining = []
    for bottom_metres, top_metres in height_ranges:
        if removed_top_metres <= bottom_metres or removed_bottom_metres >= top_metres:
            remaining.append((bottom_metres, top_metres))
            continue
        if bottom_metres < removed_bottom_metres:
            remaining.append((bottom_metres, removed_bottom_metres))
        if removed_top_metres < top_metres:
            remaining.append((removed_top_metres, top_metres))
    return remaining


# ---- The tunnel ------------------------------------------------------------------------------

def _all_tubes_sdf(course_map):
    models = []
    for tube in course_map.tubes:
        models.append(_tube_sdf(tube))
    return "\n".join(models)


def _tube_sdf(tube):
    """An oval tunnel lying on the floor, made of flat strips that run its whole length.

    Seen from the entrance, the strips sit around an oval halfway through the tunnel's wall
    thickness. Each strip is a thin box, tilted to follow the curve at its place on the oval.
    """
    axis = tube.exit_centre_metres - tube.entrance_centre_metres
    heading_radians = math.atan2(axis[1], axis[0])
    middle_of_floor_line = (tube.entrance_centre_metres + tube.exit_centre_metres) / 2.0
    # A unit vector pointing to the left of the tunnel, looking from entrance to exit.
    to_the_left = (-math.sin(heading_radians), math.cos(heading_radians))

    middle_height_metres = tube.outside_height_metres / 2.0
    oval_half_width_metres = tube.outside_width_metres / 2.0 - tube.wall_thickness_metres / 2.0
    oval_half_height_metres = tube.outside_height_metres / 2.0 - tube.wall_thickness_metres / 2.0
    oval_points = _points_around_oval(oval_half_width_metres, oval_half_height_metres, TUNNEL_STRIP_COUNT)

    shapes = []
    for number in range(TUNNEL_STRIP_COUNT):
        sideways_1, up_1 = oval_points[number]
        sideways_2, up_2 = oval_points[(number + 1) % TUNNEL_STRIP_COUNT]
        strip_middle_sideways_metres = (sideways_1 + sideways_2) / 2.0
        strip_middle_height_metres = middle_height_metres + (up_1 + up_2) / 2.0
        strip_width_metres = math.hypot(sideways_2 - sideways_1, up_2 - up_1)
        # How far the strip is turned about the tunnel's long axis, so that it follows the oval.
        tilt_radians = math.atan2(up_2 - up_1, sideways_2 - sideways_1)
        centre = (
            middle_of_floor_line[0] + to_the_left[0] * strip_middle_sideways_metres,
            middle_of_floor_line[1] + to_the_left[1] * strip_middle_sideways_metres,
            strip_middle_height_metres,
        )
        # Each strip is made one wall-thickness wider than the gap between its two oval points,
        # so neighbouring strips overlap a little and leave no cracks at their joins.
        size = (tube.length_metres(), strip_width_metres + tube.wall_thickness_metres, tube.wall_thickness_metres)
        shapes.append(_box_sdf(f"strip_{number}", centre, heading_radians, tilt_radians, size, TUNNEL_COLOUR))
    return _static_model_sdf(tube.name, shapes)


def _points_around_oval(half_width_metres, half_height_metres, point_count):
    """Evenly spaced points around an oval centred on (0, 0), as (sideways, up) pairs,
    going round anticlockwise as seen from the tunnel's entrance."""
    points = []
    for number in range(point_count):
        angle_radians = 2.0 * math.pi * number / point_count
        points.append((half_width_metres * math.cos(angle_radians), half_height_metres * math.sin(angle_radians)))
    return points


# ---- Low obstacles ---------------------------------------------------------------------------

def _all_low_obstacles_sdf(course_map):
    models = []
    for obstacle in course_map.low_obstacles:
        cylinder = (
            f"<cylinder><radius>{obstacle.radius_metres:.4f}</radius>"
            f"<length>{obstacle.top_height_metres:.4f}</length></cylinder>"
        )
        centre = (obstacle.centre_metres[0], obstacle.centre_metres[1], obstacle.top_height_metres / 2.0)
        shape = _shape_sdf("body", cylinder, centre, 0.0, 0.0, LOW_OBSTACLE_COLOUR)
        models.append(_static_model_sdf(obstacle.name, [shape]))
    return "\n".join(models)


# ---- Small SDF building blocks ---------------------------------------------------------------

def _static_model_sdf(name, shapes):
    """A model that never moves, with one part ("link") holding all its shapes. Each shape
    carries its own position in course coordinates, so the model itself sits at the course
    frame's origin."""
    return f"""    <model name="{model_name_for(name)}">
      <static>true</static>
      <pose relative_to="{COURSE_FRAME_NAME}">0 0 0 0 0 0</pose>
      <link name="link">
{"".join(shapes)}      </link>
    </model>"""


def _box_sdf(name, centre_metres, heading_radians, tilt_radians, size_metres, colour):
    size_x, size_y, size_z = size_metres
    box = f"<box><size>{size_x:.4f} {size_y:.4f} {size_z:.4f}</size></box>"
    return _shape_sdf(name, box, centre_metres, heading_radians, tilt_radians, colour)


def _shape_sdf(name, geometry_sdf, centre_metres, heading_radians, tilt_radians, colour):
    """One shape that cameras see and the physics treats as solid.

    In SDF, a pose is "x y z roll pitch yaw". Gazebo first rolls the shape about its own x
    axis (our tilt_radians), then turns it about the vertical axis (our heading_radians).
    That order is what lets a tunnel strip first tilt to follow the oval and then turn to
    point along the tunnel."""
    x, y, z = centre_metres
    pose = f"{x:.4f} {y:.4f} {z:.4f} {tilt_radians:.5f} 0 {heading_radians:.5f}"
    red, green, blue, opacity = colour
    colour_text = f"{red} {green} {blue} {opacity}"
    return f"""        <collision name="{name}_collision">
          <pose>{pose}</pose>
          <geometry>{geometry_sdf}</geometry>
        </collision>
        <visual name="{name}_visual">
          <pose>{pose}</pose>
          <geometry>{geometry_sdf}</geometry>
          <material><ambient>{colour_text}</ambient><diffuse>{colour_text}</diffuse></material>
        </visual>
"""


def model_name_for(name):
    """Gazebo names may not contain spaces or brackets, so we keep letters, digits and
    underscores, and replace everything else with an underscore. "C2a chicane wall (window)"
    becomes "C2a_chicane_wall__window_"."""
    safe_characters = []
    for character in name:
        if character.isalnum() or character == "_":
            safe_characters.append(character)
        else:
            safe_characters.append("_")
    return "".join(safe_characters)


def _escape(text):
    """Makes text safe to put inside an XML comment."""
    return text.replace("--", "- -").replace("<", "(").replace(">", ")")
