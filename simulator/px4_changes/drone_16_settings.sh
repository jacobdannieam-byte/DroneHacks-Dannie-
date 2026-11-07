#!/bin/sh
# The settings PX4 uses in the simulator. SIMULATOR ONLY.
#
# PX4 has about a thousand settings. This file copies the ones that change how the
# drone climbs, descends, lands and works out its position, so the simulated drone
# behaves as much like drone 16 as we can make it. PX4's startup script reads any
# variable named PX4_PARAM_<name> and applies it as a setting.
#
# Unless a line says otherwise, each value was read from drone 16's own settings
# file, saved on 2026-11-03. The real file has about a thousand lines; the rest are
# for hardware we are not simulating, or they do not affect these behaviours.
#
# The settings for the fast balancing loops (the "MC_" settings) are not listed,
# because drone 16 uses PX4's default value for every one of them, and so does the
# simulator. We checked this on 2026-11-07 against drone 16's file and PX4 1.16.2's
# source code. Nobody has tuned these loops for the real frame yet.

# ---- How the simulated body behaves. These are estimates, not drone 16 settings ---
# The mass is estimated from the parts list. We have not weighed the drone.
export PX4_PARAM_SIH_MASS=1.47
# Thrust per motor at full power. Estimated from the drone's hover setting below:
# if half power holds up 1.47 kg, then each of the 4 motors gives 7.21 newtons at
# full power (1.47 kg x 9.81 / 4 / 0.5).
export PX4_PARAM_SIH_T_MAX=7.21
# Stops PX4's built-in simulation of the bottom laser, which only knows about a flat
# floor. Our own pretend laser knows the course, so it can see boulders and walls.
# PX4 turns its own version off when this setting is 10000 or more.
export PX4_PARAM_SIH_DISTSNSR_OVR=20000

# ---- Where the four motors sit, in metres from the drone's balance point ------------
# From the organizers' file 07_airframe_geometry.params, measured on drone 20; they say
# every drone uses the same frame. Forward (PX) and right (PY) are positive. PX4's
# control part uses these to decide how hard each motor must push to tilt the drone.
# The physics uses the same positions, through use_drone_16_motor_positions.py.
export PX4_PARAM_CA_ROTOR0_PX=0.08        # Front right motor.
export PX4_PARAM_CA_ROTOR0_PY=0.16
export PX4_PARAM_CA_ROTOR1_PX=-0.15       # Back left motor.
export PX4_PARAM_CA_ROTOR1_PY=-0.14
export PX4_PARAM_CA_ROTOR2_PX=0.08        # Front left motor.
export PX4_PARAM_CA_ROTOR2_PY=-0.16
export PX4_PARAM_CA_ROTOR3_PX=-0.15       # Back right motor.
export PX4_PARAM_CA_ROTOR3_PY=0.14

# Tells PX4 to stamp each incoming message with its own clock at the moment it arrives.
# This differs from the real drone, on purpose. Normally PX4 converts the time on each
# message into its own time, which works on the drone because both clocks run at the
# same speed. PX4's simulated clock runs a few percent fast or slow, depending on how
# busy the Mac is. With the conversion on, that drift made PX4 reset its clock matching,
# refuse to arm, and receive sensor readings with the wrong timing (seen on 2026-11-06).
export PX4_PARAM_UXRCE_DDS_SYNCT=0

# ---- How strongly the drone chases its target position and speed ------------------
export PX4_PARAM_MPC_XY_P=0.95            # Sideways: target speed per metre of error.
export PX4_PARAM_MPC_Z_P=1.0              # Same, up and down.
export PX4_PARAM_MPC_XY_VEL_P_ACC=1.8     # Sideways: acceleration per m/s of speed error.
export PX4_PARAM_MPC_XY_VEL_I_ACC=0.4     # Sideways: slow correction for steady error.
export PX4_PARAM_MPC_XY_VEL_D_ACC=0.2     # Sideways: damping.
export PX4_PARAM_MPC_Z_VEL_P_ACC=4.0      # Up and down: acceleration per m/s of speed error.
export PX4_PARAM_MPC_Z_VEL_I_ACC=2.0      # Up and down: slow correction. This is what
                                          # lowers motor power after touching the floor.
export PX4_PARAM_MPC_Z_VEL_D_ACC=0.0      # Up and down: damping (none).

# ---- Motor power limits, as a fraction of full power -------------------------------
export PX4_PARAM_MPC_THR_HOVER=0.5        # Expected power needed to hover.
export PX4_PARAM_MPC_THR_MIN=0.12         # Lowest power while flying under control.
export PX4_PARAM_MPC_THR_MAX=1.0          # Highest power.
export PX4_PARAM_MPC_MANTHR_MIN=0.08      # Lowest power when flown by hand.
export PX4_PARAM_MPC_USE_HTE=1            # Let PX4 learn the real hover power in flight.

# ---- Speed and acceleration limits --------------------------------------------------
export PX4_PARAM_MPC_XY_VEL_MAX=12.0      # Sideways speed limit, m/s.
export PX4_PARAM_MPC_Z_VEL_MAX_DN=1.5     # Downward speed limit, m/s.
export PX4_PARAM_MPC_Z_VEL_MAX_UP=3.0     # Upward speed limit, m/s.
export PX4_PARAM_MPC_ACC_HOR=3.0          # Sideways acceleration limit, m/s^2.
export PX4_PARAM_MPC_ACC_DOWN_MAX=3.0     # Downward acceleration limit, m/s^2.
export PX4_PARAM_MPC_ACC_UP_MAX=4.0       # Upward acceleration limit, m/s^2.
export PX4_PARAM_MPC_JERK_AUTO=4.0        # How quickly acceleration may change.
export PX4_PARAM_MPC_TILTMAX_AIR=45.0     # Steepest allowed tilt, degrees.

# ---- When PX4 decides the drone has landed ------------------------------------------
export PX4_PARAM_LNDMC_Z_VEL_MAX=0.25     # Must be moving down slower than this, m/s.
export PX4_PARAM_LNDMC_XY_VEL_MAX=1.5     # Must be moving sideways slower than this, m/s.
export PX4_PARAM_LNDMC_ROT_MAX=20.0       # Must be rotating slower than this, degrees/s.
export PX4_PARAM_LNDMC_TRIG_TIME=1.0      # All of it must stay true for this long, seconds.
export PX4_PARAM_LNDMC_ALT_GND=2.0        # Ground-check height limit, metres.

# ---- When PX4 switches the motors off, and what happens if commands stop ------------
export PX4_PARAM_COM_DISARM_LAND=2.0      # Seconds after landing before motors stop.
export PX4_PARAM_COM_DISARM_PRFLT=10.0    # Seconds to take off after arming, or motors stop.
export PX4_PARAM_COM_OF_LOSS_T=1.0        # Seconds without commands before automatic flight ends.
export PX4_PARAM_COM_OBL_RC_ACT=0         # What to do when automatic flight ends (Position mode).
export PX4_PARAM_COM_RC_IN_MODE=0         # Hand control comes from the radio only.
export PX4_PARAM_COM_RC_OVERRIDE=1        # Moving the sticks takes back control.
export PX4_PARAM_COM_ARM_WO_GPS=1         # Allowed to start without GPS.
export PX4_PARAM_NAV_RCL_ACT=2            # If the radio link is lost, land.

# ---- How PX4 works out its position (EKF2 is the part of PX4 that does this) --------
export PX4_PARAM_EKF2_GPS_CTRL=0          # Ignore GPS. There is none indoors.
export PX4_PARAM_EKF2_OF_CTRL=0           # No optical flow sensor.
export PX4_PARAM_EKF2_EV_CTRL=9           # Use the camera tracking for position and heading.
export PX4_PARAM_EKF2_EV_NOISE_MD=0       # Trust the error size that the camera tracking reports.
export PX4_PARAM_EKF2_EV_DELAY=60.0       # Camera tracking reports arrive this late, milliseconds.
export PX4_PARAM_EKF2_EVP_NOISE=0.1       # Smallest assumed position error, metres.
export PX4_PARAM_EKF2_EVA_NOISE=0.05      # Smallest assumed heading error, radians.
export PX4_PARAM_EKF2_EVV_NOISE=0.1       # Smallest assumed speed error, m/s.
export PX4_PARAM_EKF2_EV_QMIN=0           # Accept camera tracking at any reported quality.
export PX4_PARAM_EKF2_EVP_GATE=5.0        # How large a position surprise is still believed.
export PX4_PARAM_EKF2_EVV_GATE=3.0        # How large a speed surprise is still believed.
export PX4_PARAM_EKF2_HGT_REF=2           # The bottom laser is the main source of height.
export PX4_PARAM_EKF2_BARO_CTRL=1         # The air pressure sensor is also used.
export PX4_PARAM_EKF2_RNG_CTRL=1          # Use the bottom laser only while the two
                                          # conditions below hold. Otherwise PX4 uses
                                          # the air pressure sensor for height.
export PX4_PARAM_EKF2_RNG_A_VMAX=1.0      # Condition 1: moving sideways slower than this, m/s.
export PX4_PARAM_EKF2_RNG_A_HMAX=3.0      # Condition 2: lower than this above the floor, m.
export PX4_PARAM_EKF2_RNG_NOISE=0.010058  # Expected laser error, metres.
export PX4_PARAM_EKF2_RNG_SFE=0.05        # Extra expected error per metre of distance.
export PX4_PARAM_EKF2_RNG_GATE=5.0        # How large a laser surprise is still believed.
export PX4_PARAM_EKF2_RNG_POS_Y=-0.027432 # The laser sits this far to the left of centre, m.
export PX4_PARAM_EKF2_RNG_POS_Z=0.03445   # The laser sits this far below the drone's centre.
export PX4_PARAM_EKF2_RNG_DELAY=5.0       # How late PX4 assumes laser readings are, ms.
                                          # The real lag is nearer 100 ms (see
                                          # pretend_bottom_laser.py). We copy the drone's
                                          # value anyway, because we want the same mistake.
export PX4_PARAM_EKF2_MIN_RNG=0.1         # The expected laser reading when on the ground.
export PX4_PARAM_EKF2_NOAID_TOUT=5000000  # Microseconds of no position input before giving up.
export PX4_PARAM_EKF2_MAG_TYPE=0          # Choose automatically how to use the compass.
