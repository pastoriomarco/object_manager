# Object Manager

![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)
![ROS](https://img.shields.io/badge/ROS-2%20Humble-orange.svg)

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Installation](#installation)
- [Usage](#usage)
  - [Launching the Package](#launching-the-package)
  - [Service Interfaces](#service-interfaces)
- [Configuration](#configuration)
- [Visualization](#visualization)
- [Dependencies](#dependencies)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Overview

The **Object Manager** is a ROS 2 package designed to manage collision objects within a robotic environment using MoveIt. It provides functionalities to add, remove, and manage collision objects dynamically, ensuring a clean and controlled workspace for robotic applications.

## Features

- **Add Collision Objects:** Easily add various shapes (box, cylinder, sphere) as collision objects to the planning scene.
- **Remove Collision Objects:** Remove specific or all previously added collision objects.
- **Automated Spawning:** Automatically remove existing spawner objects and spawn new ones with designated postfixes.
- **Configurability:** Customize frame IDs and object parameters through ROS 2 parameters.
- **Integration with MoveIt:** Leverages MoveIt's Planning Scene Interface for seamless integration with motion planning.

## Architecture

The package consists of two primary nodes:

1. **Object Manager Node (`object_manager_node`):**
   - **Services Provided:**
     - `/add_collision_object`: Adds a collision object to the scene.
     - `/remove_collision_object`: Removes a specific collision object from the scene.
     - `/remove_all_objects`: Removes all collision objects managed by this node.
   - **Functionality:** Manages the lifecycle of collision objects, tracking added objects and ensuring proper addition and removal.

2. **Collision Spawner Node (`collision_spawner`):**
   - **Functionality:**
     - Upon initialization, it removes all existing spawner objects (objects with IDs ending in `_spawner`).
     - Spawns new collision objects with `_spawner` postfixes to maintain a clean environment.
   - **Objects Spawned:**
     - **Obstacle:** A box-shaped obstacle.
     - **Graspable Cylinder:** A randomly positioned and oriented cylinder.

## Installation

### Prerequisites

- **ROS 2:** Ensure you have ROS 2 (Humble Hawksbill or later) installed.
- **MoveIt 2:** Install MoveIt 2 for motion planning capabilities.

### Clone the Repository

```bash
cd ~/workspaces/ros2_ws/src
git clone https://github.com/yourusername/object_manager.git
