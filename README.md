TERMINAL 1 — BACKEND
==================================================

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/cognipilot/cranium/install/setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/cognipilot/cranium/install/dream_world/share/dream_world/models:$HOME/cognipilot/cranium/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

ros2 launch b3rb_gz_bringup sil.launch.py world:=nxp_raceway_octagon track_vision:=false

==================================================
TERMINAL 2 — GAZEBO GUI
==================================================

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/cognipilot/cranium/install/setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/cognipilot/cranium/install/dream_world/share/dream_world/models:$HOME/cognipilot/cranium/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

gz sim -g --force-version 8

==================================================
TERMINAL 3 — VECTORS
==================================================

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/cognipilot/cranium/install/setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/cognipilot/cranium/install/dream_world/share/dream_world/models:$HOME/cognipilot/cranium/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

ros2 run b3rb_ros_line_follower vectors

==================================================
TERMINAL 4 — RUNNER
==================================================

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/cognipilot/cranium/install/setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/cognipilot/cranium/install/dream_world/share/dream_world/models:$HOME/cognipilot/cranium/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

ros2 run b3rb_ros_line_follower runner_mrac

==================================================
TERMINAL 5 — DETECT
==================================================

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/cognipilot/cranium/install/setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/cognipilot/cranium/install/dream_world/share/dream_world/models:$HOME/cognipilot/cranium/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

ros2 run b3rb_ros_line_follower detect

==================================================
TERMINAL 6 — CAMERA VIEWER
==================================================

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/cognipilot/cranium/install/setup.bash"

python3 ~/debug_compressed_viewer.py
