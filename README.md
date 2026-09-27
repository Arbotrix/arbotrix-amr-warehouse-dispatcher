# Arbotrix Autonomous Warehouse AMR Dispatcher

Autonomous Warehouse Mobile Robot developed using ROS 2 Humble,
Nav2, AMCL, Gazebo Classic and a web-based ROSBridge dashboard.

## Main Features

- ROS 2 Humble
- Ubuntu 22.04
- Gazebo Classic warehouse simulation
- Differential-drive AMR
- LiDAR and RGB camera
- AMCL localization
- Nav2 autonomous navigation
- Global and local costmaps
- Pickup Stations A, B and C
- Drop Zones 1, 2 and 3
- Autonomous dispatcher
- Web-based dispatcher dashboard
- ROSBridge WebSocket communication
- Mission queue
- Live distance remaining
- Mission history

## Warehouse Coordinates

### Pickup Stations

- Station A: (-4.0, 4.0)
- Station B: (0.0, 4.0)
- Station C: (4.0, 4.0)

### Drop Zones

- Zone 1: (-4.0, -4.0)
- Zone 2: (0.0, -4.0)
- Zone 3: (4.0, -4.0)

### Home Base

- Home: (0.0, -3.5)

## Packages

### arbotrix_sim

Contains:

- Robot URDF
- Gazebo world
- Nav2 configuration
- AMCL configuration
- Warehouse map
- RViz configuration
- Simulation launch files

### warehouse_dispatcher

Contains:

- Autonomous dispatcher
- Web dispatcher
- ROSBridge web interface
- Mission queue logic
- Navigation action client
- Browser dashboard
