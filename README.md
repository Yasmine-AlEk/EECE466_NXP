!Terminal Commands For The New Repo!
Terminal 1 Backend

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/EECE466_NXP_publish/install/local_setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/EECE466_NXP_publish/install/dream_world/share/dream_world/models:$HOME/EECE466_NXP_publish/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

ros2 launch b3rb_gz_bringup sil.launch.py world:=nxp_raceway_octagon track_vision:=false

Terminal 2 Gazebo GUI

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/EECE466_NXP_publish/install/local_setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/EECE466_NXP_publish/install/dream_world/share/dream_world/models:$HOME/EECE466_NXP_publish/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

gz sim -g --force-version 8

Terminal 3 Vectors

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/EECE466_NXP_publish/install/local_setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/EECE466_NXP_publish/install/dream_world/share/dream_world/models:$HOME/EECE466_NXP_publish/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

ros2 run b3rb_ros_line_follower vectors

Terminal 4 Runner

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/EECE466_NXP_publish/install/local_setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/EECE466_NXP_publish/install/dream_world/share/dream_world/models:$HOME/EECE466_NXP_publish/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

ros2 run b3rb_ros_line_follower runner_mrac

Terminal 5 Detect

source /opt/ros/humble/setup.bash
[ -f "$HOME/cognipilot/ws/cerebri/install/setup.sh" ] && source "$HOME/cognipilot/ws/cerebri/install/setup.sh"
[ -f "$HOME/cognipilot/gazebo/install/setup.sh" ] && source "$HOME/cognipilot/gazebo/install/setup.sh"
source "$HOME/EECE466_NXP_publish/install/local_setup.bash"

export GZ_SIM_RESOURCE_PATH=$HOME/EECE466_NXP_publish/install/dream_world/share/dream_world/models:$HOME/EECE466_NXP_publish/src/dream_world/models:$GZ_SIM_RESOURCE_PATH
export LIBGL_ALWAYS_SOFTWARE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
export GZ_RENDER_ENGINE=ogre
export GZ_PARTITION=nxpfull

ros2 run b3rb_ros_line_follower detect
