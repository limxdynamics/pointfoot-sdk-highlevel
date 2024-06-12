cmake_minimum_required(VERSION 3.5)

project(pointfoot_highlevel)

set(CMAKE_BUILD_TYPE "Release")

include_directories(${PROJECT_SOURCE_DIR}/include)

if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_VERSION} EQUAL 1)
    # In ROS compilation environment
    message("Currently in ROS compilation environment")
    
    find_package(catkin REQUIRED COMPONENTS 
      roscpp
      geometry_msgs
      sensor_msgs
      diagnostic_msgs
    )

    add_custom_target(pointfoot_highlevel_include_target ALL
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/include/. ${CATKIN_DEVEL_PREFIX}/include
    )

    if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
      set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
      add_custom_target(pointfoot_highlevel_target ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/libpointfoot_highlevel.so ${CATKIN_DEVEL_PREFIX}/lib
      )
    elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
      set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
      add_custom_target(pointfoot_highlevel_target ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/libpointfoot_highlevel.so ${CATKIN_DEVEL_PREFIX}/lib
      )
    elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm")
      set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
      add_custom_target(pointfoot_highlevel_target ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/libpointfoot_highlevel.so ${CATKIN_DEVEL_PREFIX}/lib
      )
    elseif (WIN32)
      set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
      add_custom_target(pointfoot_highlevel_target ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/pointfoot_highlevel.dll ${CATKIN_DEVEL_PREFIX}/lib
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/pointfoot_highlevel.lib ${CATKIN_DEVEL_PREFIX}/lib
      )
    else()
      message(FATAL_ERROR "Unsupported architecture!")
    endif()

    catkin_package(
      INCLUDE_DIRS include
      CATKIN_DEPENDS
        roscpp
        sensor_msgs
        geometry_msgs
        diagnostic_msgs
    )

    include_directories(${catkin_INCLUDE_DIRS})

    link_directories( ${LIB_PATH} )

    add_executable(pointfoot_highlevel_node ${PROJECT_SOURCE_DIR}/src/pointfoot_highlevel_node.cpp)

    target_link_libraries(pointfoot_highlevel_node  pointfoot_highlevel ${catkin_LIBRARIES})

    install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    
    # Install library files to build output
    install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})

    # Install launch files to build output
    install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

    # Install pointfoot_highlevel_node files to build output
    install(TARGETS pointfoot_highlevel_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  elseif($ENV{ROS_VERSION} EQUAL 2)
    # In ROS2 compilation environment
    message("Currently in ROS2 compilation environment")

    find_package(ament_cmake REQUIRED)

    if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
      set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
    elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
      set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
    elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm")
      set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
    elseif (WIN32)
      set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
    else()
      message(FATAL_ERROR "Unsupported architecture!")
    endif()

    link_directories( ${LIB_PATH} )

    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
    
    # Install library files to build output
    install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

    # Export the library
    ament_export_libraries(pointfoot_highlevel)

    # Export the include
    ament_export_include_directories(include)

    ament_package()
  endif()
else()
  # In CMake compilation environment
  message("Currently in CMake compilation environment")

  if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
  elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
  elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
  elseif (WIN32)
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
    install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
  else()
    message(FATAL_ERROR "Unsupported architecture!")
  endif()

  link_directories(${LIB_PATH})

  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${CMAKE_INSTALL_PREFIX})
  
  # Install library files to build output
  install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
  
endif()
