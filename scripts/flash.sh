#!/bin/bash

# Flash the board

# verify that this is running in the project root
if [ ! -f "project.yml" ]; then
  echo "Error: must be run from the project root."
  echo "Note: this script is looking for the project.yml file to determine what directory it's called from."
  exit 1
fi

# Create a variable defining the project root that we're currently in
PROJECT_DIR=$(pwd)
BUILD_DIR="$PROJECT_DIR/build/Debug"

# Create a variable with the project name from the CMakeLists.txt file
PROJECT_NAME=$(grep "project(" CMakeLists.txt | cut -d"(" -f2 | cut -d" " -f1 | tr -d ")")

# Print the project name with the *.hex extension
echo "Flashing $PROJECT_NAME.hex"

# Command to flash the board with the STM32_Programmer_CLI
STM32_Programmer_CLI --connect port=swd --download $BUILD_DIR/$PROJECT_NAME.hex -hardRst

