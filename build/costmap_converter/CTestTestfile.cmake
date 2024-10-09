# CMake generated Testfile for 
# Source directory: /home/jetson/slash_ws/src/slash_navigation/costmap_converter/costmap_converter
# Build directory: /home/jetson/slash_ws/build/costmap_converter
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_costmap_polygons "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/jetson/slash_ws/build/costmap_converter/test_results/costmap_converter/test_costmap_polygons.gtest.xml" "--package-name" "costmap_converter" "--output-file" "/home/jetson/slash_ws/build/costmap_converter/ament_cmake_gtest/test_costmap_polygons.txt" "--command" "/home/jetson/slash_ws/build/costmap_converter/test_costmap_polygons" "--gtest_output=xml:/home/jetson/slash_ws/build/costmap_converter/test_results/costmap_converter/test_costmap_polygons.gtest.xml")
set_tests_properties(test_costmap_polygons PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/jetson/slash_ws/build/costmap_converter/test_costmap_polygons" TIMEOUT "60" WORKING_DIRECTORY "/home/jetson/slash_ws/build/costmap_converter" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/jetson/slash_ws/src/slash_navigation/costmap_converter/costmap_converter/CMakeLists.txt;90;ament_add_gtest;/home/jetson/slash_ws/src/slash_navigation/costmap_converter/costmap_converter/CMakeLists.txt;0;")
subdirs("gtest")
