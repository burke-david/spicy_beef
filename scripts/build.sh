#!/bin/bash

# Build the project

# verify that this is running in the project root
if [ ! -f "project.yml" ]; then
  echo "Error: must be run from the project root."
  echo "Note: this script is looking for the project.yml file to determine what directory it's called from."
  exit 1
fi

# Create a variable defining the project root that we're currently in
PROJECT_DIR=$(pwd)
BUILD_DIR="$PROJECT_DIR/build/Debug"
TOOLCHAIN_FILE="$PROJECT_DIR/cmake/gcc-arm-none-eabi.cmake"

cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE -S$PROJECT_DIR -B$BUILD_DIR -G Ninja

# Add the cmake command to build the project
cmake --build $BUILD_DIR

#cmake --build ${command:cmake.buildDirectory}

#if the build directory does not exist, create it
#if [ ! -d "build" ]; then
#  mkdir build
#fi

# Run cmake to generate the build files
#cd build
#cmake ..

# Run make to build the project
#make
