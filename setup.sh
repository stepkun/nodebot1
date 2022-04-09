#!/bin/sh
# change into parent dir (src of ros2 workspace)
cd ..
# install pico-chassis and its dependencies
if [ -d pico-chassis ]
then
    echo "The pico_chassis is already installed"
    echo "To reinstall remove directory pico_chassis in your ros2 workspace src directory"
else
    git clone git@github.com:stepkun/pico_chassis.git
    cd pico_chassis
    ./setup_micro-ros-pico-sdk.sh
    ./setup_micro-ros-agent.sh
    cd ..
fi

# change into parent workspace dir and update ros dependencies
cd ..
rosdep update
rosdep install -i --from-path src --rosdistro galactic -y
