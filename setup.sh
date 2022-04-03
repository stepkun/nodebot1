#!/bin/sh
# change into parent src dir
cd ..
# install pico-chassis and its dependencies
if [ -d pico-chassis ]
then
    echo "The pico-chassis is already installed"
    echo "To reinstall remove directory pico-chassis in your ros2 workspace src directory"
else
    git clone git@github.com:stepkun/pico-chassis.git
    cd pico-chassis
    ./setup_micro-ros-pico-sdk.sh
    ./setup_micro-ros-agent.sh
    cd ..
fi

# change into parent workspace dir and update ros dependencies
cd ..
rosdep update
rosdep install -i --from-path src --rosdistro galactic -y
