# KMR Loader

The KMR Loader is an extension to load the KUKA KMR iiwa into an environment in Isaac Sim. A URDF description of the robot has to be provided for the extension to work. A ROS 2 stack for the KMR has been developed with a description and is located in [this repo](https://github.com/jorgenmyrvold/kmr_ws)

**Requirements**
- ROS 2 Foxy
- Only tested on Isaac Sim 2022.1.1
- URDF model of the KMR ([this repo](https://github.com/jorgenmyrvold/kmr_ws))
- Modified wheels according to [this](#configure-omniwheels)

## Usage
- The path of the URDF description has to be set in the [source code](../omni/isaac/kmr_loader/kmr_loader.py)
- Reset the scene by clicking *File -> New From Stage Template -> Empty*
- Select environment for the robot to be placed in
- Check or uncheck box for *Publish camers* (Disabled by default for performance reasons)
- Click *LOAD*

Then the robot should appear in the stage and when running it should be possible to control the robot using ROS 2 packages such as [teleop_twist_keyboard](https://index.ros.org/r/teleop_twist_keyboard/github-ros2-teleop_twist_keyboard/#foxy)

## Configure omniwheels
In some cases the wheels included in the [data](../data/) directory does not work propperly when imported as referenecs. If this is the case use the following workaround

1. Clone [O3dynSimModel repo](https://git.openlogisticsfoundation.org/silicon-economy/simulation-model/o3dynsimmodel) or copy `wheel_right.usd` and `wheel_left.usd` from `omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Robots/O3dyn/parts/` into a arbitrary directory.
2. Create two copies of `wheel_right.usd` and `wheel_left.usd`, and call the
`wheel_right.usd` copies `wheel_fr.usd` and `wheel_rl.usd`, and the copies
of `wheel_left.usd` `wheel_fl.usd` and `wheel_rr.usd`.
3. Open each of the four new files in Isaac Sim and rename the Rollers from Roller_i (with i from 1-7) for Roller_i_[position] and the core named omniwheel to omni-
wheel_[position] (where position is the last two characters in the filename.
without brackets, fr, rl, fl or rr).
4. Specify the path to the wheel usd files in kmr_loader.py in the omni.isaac.kmr_loader extension in [kmr_loader.py](../omni/isaac/kmr_loader/kmr_loader.py).

The reason for this workaround is so that the automatically published ROS 2 tf messages uses unique frame ids and that the frames of the rollers does not jump between different positions as the wheels names are identical.
