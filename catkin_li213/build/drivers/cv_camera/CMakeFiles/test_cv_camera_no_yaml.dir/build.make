# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/youbot/catkin_li213/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/youbot/catkin_li213/build

# Include any dependencies generated for this target.
include drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/depend.make

# Include the progress variables for this target.
include drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/progress.make

# Include the compile flags for this target's objects.
include drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/flags.make

drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o: drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/flags.make
drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o: /home/youbot/catkin_li213/src/drivers/cv_camera/test/test_cv_camera_no_yaml.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/youbot/catkin_li213/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o"
	cd /home/youbot/catkin_li213/build/drivers/cv_camera && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o -c /home/youbot/catkin_li213/src/drivers/cv_camera/test/test_cv_camera_no_yaml.cpp

drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.i"
	cd /home/youbot/catkin_li213/build/drivers/cv_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/youbot/catkin_li213/src/drivers/cv_camera/test/test_cv_camera_no_yaml.cpp > CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.i

drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.s"
	cd /home/youbot/catkin_li213/build/drivers/cv_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/youbot/catkin_li213/src/drivers/cv_camera/test/test_cv_camera_no_yaml.cpp -o CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.s

drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o.requires:
.PHONY : drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o.requires

drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o.provides: drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o.requires
	$(MAKE) -f drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/build.make drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o.provides.build
.PHONY : drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o.provides

drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o.provides.build: drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o

# Object files for target test_cv_camera_no_yaml
test_cv_camera_no_yaml_OBJECTS = \
"CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o"

# External object files for target test_cv_camera_no_yaml
test_cv_camera_no_yaml_EXTERNAL_OBJECTS =

/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/build.make
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: gtest/libgtest.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /home/youbot/catkin_li213/devel/lib/libcv_camera.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libimage_transport.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libmessage_filters.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libcv_bridge.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libnodeletlib.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libbondcpp.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libuuid.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libtinyxml.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libclass_loader.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/libPocoFoundation.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libdl.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libroslib.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libcamera_info_manager.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libroscpp.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libboost_signals.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libboost_filesystem.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/librosconsole.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/liblog4cxx.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libboost_regex.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/librostime.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libboost_date_time.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /opt/ros/indigo/lib/libcpp_common.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libconsole_bridge.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libboost_system.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libboost_thread.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libpthread.so
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml: drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml"
	cd /home/youbot/catkin_li213/build/drivers/cv_camera && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_cv_camera_no_yaml.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/build: /home/youbot/catkin_li213/devel/lib/cv_camera/test_cv_camera_no_yaml
.PHONY : drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/build

drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/requires: drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/test/test_cv_camera_no_yaml.cpp.o.requires
.PHONY : drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/requires

drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/clean:
	cd /home/youbot/catkin_li213/build/drivers/cv_camera && $(CMAKE_COMMAND) -P CMakeFiles/test_cv_camera_no_yaml.dir/cmake_clean.cmake
.PHONY : drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/clean

drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/depend:
	cd /home/youbot/catkin_li213/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/youbot/catkin_li213/src /home/youbot/catkin_li213/src/drivers/cv_camera /home/youbot/catkin_li213/build /home/youbot/catkin_li213/build/drivers/cv_camera /home/youbot/catkin_li213/build/drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : drivers/cv_camera/CMakeFiles/test_cv_camera_no_yaml.dir/depend

