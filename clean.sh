#!/bin/bash

# Clean ROS build and development files
echo "Cleaning ROS workspace..."
rm -rf build/
rm -rf devel/
rm -rf install/
rm -rf logs/
echo "Cleaning CMake/catkin files..."
find . -name "CMakeCache.txt" -type f -delete
find . -name "CMakeFiles" -type d -exec rm -rf {} +
find . -name "Makefile" -type f -delete
find . -name "catkin*" -type f -delete
find . -name "*.cmake" -type f -delete
find . -name "CMakeLists.txt.user" -type f -delete

echo "Cleaning complete!"