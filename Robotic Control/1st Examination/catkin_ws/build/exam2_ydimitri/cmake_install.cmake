# Install script for directory: /home/ydimitri/Desktop/Exams_1/catkin_ws/src/exam2_ydimitri

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ydimitri/Desktop/Exams_1/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/exam2_ydimitri/action" TYPE FILE FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/src/exam2_ydimitri/action/Turtle.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/exam2_ydimitri/msg" TYPE FILE FILES
    "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/exam2_ydimitri/msg/TurtleAction.msg"
    "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/exam2_ydimitri/msg/TurtleActionGoal.msg"
    "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/exam2_ydimitri/msg/TurtleActionResult.msg"
    "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/exam2_ydimitri/msg/TurtleActionFeedback.msg"
    "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/exam2_ydimitri/msg/TurtleGoal.msg"
    "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/exam2_ydimitri/msg/TurtleResult.msg"
    "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/exam2_ydimitri/msg/TurtleFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/exam2_ydimitri/cmake" TYPE FILE FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/build/exam2_ydimitri/catkin_generated/installspace/exam2_ydimitri-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/include/exam2_ydimitri")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/roseus/ros/exam2_ydimitri")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/common-lisp/ros/exam2_ydimitri")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/share/gennodejs/ros/exam2_ydimitri")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/lib/python2.7/dist-packages/exam2_ydimitri")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/devel/lib/python2.7/dist-packages/exam2_ydimitri")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/build/exam2_ydimitri/catkin_generated/installspace/exam2_ydimitri.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/exam2_ydimitri/cmake" TYPE FILE FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/build/exam2_ydimitri/catkin_generated/installspace/exam2_ydimitri-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/exam2_ydimitri/cmake" TYPE FILE FILES
    "/home/ydimitri/Desktop/Exams_1/catkin_ws/build/exam2_ydimitri/catkin_generated/installspace/exam2_ydimitriConfig.cmake"
    "/home/ydimitri/Desktop/Exams_1/catkin_ws/build/exam2_ydimitri/catkin_generated/installspace/exam2_ydimitriConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/exam2_ydimitri" TYPE FILE FILES "/home/ydimitri/Desktop/Exams_1/catkin_ws/src/exam2_ydimitri/package.xml")
endif()

