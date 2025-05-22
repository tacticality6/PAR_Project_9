#!/bin/bash

# === CONFIGURATION ===
ROS2_VERSION="humble"

# === PARSE FLAGS ===
if [ "$1" == "--clean" ]; then
    CLEAN_ONLY=true
else 
    CLEAN_ONLY=false
    LAUNCH_FILE=$1
fi   


#automatically determine workspace
# Check if the ROS 2 environment is sourced
if [ -n "$COLCON_PREFIX_PATH" ]; then
    # If sourced, use COLCON_PREFIX_PATH to find the workspace
    WS_DIR=$(dirname $(echo $COLCON_PREFIX_PATH | cut -d':' -f1))
    echo "ROS 2 Workspace detected (from COLCON_PREFIX_PATH): $WS_DIR"
elif [ -n "$AMENT_PREFIX_PATH" ]; then
    # If sourced, use AMENT_PREFIX_PATH to find the workspace
    WS_DIR=$(dirname $(echo $AMENT_PREFIX_PATH | cut -d':' -f1))
    echo "ROS 2 Workspace detected (from AMENT_PREFIX_PATH): $WS_DIR"
else
    # If not sourced, search upwards for 'src' folder
    echo "ROS 2 workspace not sourced, searching directories..."
    while [ "$PWD" != "/" ]; do
        if [ -d "$PWD/src" ]; then
            echo "Found ROS 2 workspace at: $PWD"
            break
        fi
        cd ..
    done
    if [ "$PWD" == "/" ]; then
        echo "Error: ROS 2 workspace not found."
        exit 1
    fi
    WS_DIR=$PWD
fi


# === SOURCE ROS 2 ENVIRONMENT ===
source /opt/ros/$ROS2_VERSION/setup.bash
source $WS_DIR/install/setup.bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


# === CLEANUP SECTION ===
if [ "$CLEAN_ONLY" = true ]; then
    echo "Cleaning up: deleting script directory and related build/install folders..."
    for PKG in $PACKAGE_NAMES; do
        rm -rf "$WS_DIR/build/$PKG" "$WS_DIR/install/$PKG"
        echo "Deleted build/install folders for package: $PKG"
    done
    echo "Deleting script directory: $SCRIPT_DIR"
    rm -rf "$SCRIPT_DIR"
    echo "Cleanup complete."
    exit 0
fi   


# === BUILD ALL PACKAGES IN SCRIPT DIRECTORY ===
echo "Building all packages in: $SCRIPT_DIR"
colcon build --base-paths "$SCRIPT_DIR"    

# === SOURCE NEW BUILD ===
source $WS_DIR/install/setup.bash

# === RUN LAUNCH FILE IF PROVIDED ===
LAUNCH_FILE=$1
if [ -n "$LAUNCH_FILE" ]; then
    echo "Running launch file: $LAUNCH_FILE..."
    ros2 launch $PACKAGE_NAME $LAUNCH_FILE
fi