# Code that runs on the real drone

Everything in this folder is copied to drone 16 and runs there. Simulator code lives in
`simulator/` and must never be imported from here.

## What is here today

`team_nav` is our team's ROS package. It replaces the organizers' demo, which would climb to 1.5 m
and fly a 5 m square into the course walls as soon as someone armed the drone and flipped the
switch to Position.

It now contains our mission program (see [`team_nav/README.md`](team_nav/README.md)), but the drone
**does not run it yet**. The launch file still starts only the organizers' flight interface, with
the hand-over to software switched off. The team approves the mission before anyone changes that.

Read these two files, in this order:

1. [`team_nav/launch/team_nav.launch.py`](team_nav/launch/team_nav.launch.py) says what starts on
   the drone.
2. [`install_on_drone_16.sh`](install_on_drone_16.sh) copies the package onto the drone.

## How to install it on the drone

Run these from the top of the repository, with the drone connected by USB.

Step 1. Copy the package onto the drone. The script first checks that it is really talking to
drone 16, because logging in to another team's drone gets us disqualified.

```bash
drone_code/install_on_drone_16.sh
```

Step 2. Tell the drone to start our package instead of the demo. This changes a protected file,
so it asks for the shared drone password. Please type it yourself. It must never be saved in any
file. You need to do this only once.

```bash
ssh -t dronehacks@192.168.55.1 "sudo sed -i 's/^COMPETITOR_PKG=.*/COMPETITOR_PKG=team_nav/; s/^COMPETITOR_LAUNCH=.*/COMPETITOR_LAUNCH=team_nav.launch.py/' /etc/skybeta/drone.env"
```

Step 3. Restart the drone's software so that the change takes effect. It comes back by itself
about 10 seconds later. The organizers' guide also says to power-cycle the flight controller after
stopping this software, until we have confirmed it runs their patched firmware.

```bash
ssh dronehacks@192.168.55.1 "docker rm -f skybeta-stack"
```

## How to check that it worked

The demo program should be gone, and the organizers' interface should be running:

```bash
ssh dronehacks@192.168.55.1 "docker exec skybeta-stack pgrep -af 'square_mission|flight_interface'"
```

You should see `flight_interface` in the output and no `square_mission`.
