# Manuel Robot Control (Pinocchio + Meshcat)

Interactive forward and inverse kinematics control of the **Manuel**
6-DOF manipulator using:

-   Pinocchio (rigid body kinematics)
-   Meshcat (3D visualization)
-   Python 3

------------------------------------------------------------------------

## Requirements

-   Python ≥ 3.9
-   Linux / macOS recommended

------------------------------------------------------------------------

## Installation

Clone the repository:

``` bash
git clone https://github.com/Capynetics/control_for_manuel-1.0.git
cd control_for_manuel-1.0
```

Create a virtual environment:

``` bash
python3 -m venv .venv
source .venv/bin/activate
```

Upgrade pip:

``` bash
pip install --upgrade pip
```

Install the project (dependencies are defined in `pyproject.toml`):

``` bash
pip install -e .
```

This installs:

-   numpy
-   pin (Pinocchio)
-   meshcat

------------------------------------------------------------------------

## Running the Robot Console

Start the interactive console:

``` bash
python3 -i robot_console.py
```

A browser window will open with the Meshcat 3D viewer.

You are now inside a Python interactive session with:

-   `model`
-   `data`
-   `q` (current configuration)
-   `viz`
-   `fk()`
-   `ik_clik()`

available.

------------------------------------------------------------------------

## Usage

### Forward Kinematics

Move joints smoothly:

``` python
fk([0.5, 0.2, 1.0, 0, 0, 0])
```

Check current end-effector pose:

``` python
fk()
```

------------------------------------------------------------------------

### Inverse Kinematics (Position Only)

Move the end-effector to a Cartesian position:

``` python
ik_clik("gripper_link", [0.3, 0.2, 0.4])
```

------------------------------------------------------------------------

### Inverse Kinematics with Orientation

``` python
import pinocchio as pin
import numpy as np

R = pin.rpy.rpyToMatrix(0, 0, np.pi/2)
ik_clik("gripper_link", [0.3, 0.2, 0.4], R)
```

------------------------------------------------------------------------

## Project Structure

    .
    ├── manuel/
    │   └── manuel1.urdf
    ├── robot_console.py
    ├── pyproject.toml
    └── README.md

------------------------------------------------------------------------

## Notes

-   This project performs kinematic control only.
-   No dynamics or torque control is implemented.
-   Joint limits are defined in the URDF.
-   Linking between the simulation and the real robot will be done in the future.

## Important commentary

One of Manuel’s main advantages over other open-source arms is its use of Dynamixel actuators, which enable advanced torque control strategies such as gravity compensation and impedance control. However, these methods require an accurate dynamic model. Currently, the URDF lacks inertial tags, making it impossible to fully utilize the robot’s torque control capabilities.