cmake_minimum_required(VERSION 2.8.3)
project(can_interface)

find_package(catkin REQUIRED
  COMPONENTS
    class_loader
)

find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    can_interface_string
  CATKIN_DEPENDS
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# can_interface_string
add_library(can_interface_string
    src/string.cpp
)

# socketcan_dump
add_executable(socketcan_dump
  src/socketcan_dump.cpp
)

target_link_libraries(socketcan_dump
   can_interface_string
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

# canlib_dump
# add_executable(canlib_dump
#   src/canlib_dump.cpp
# )

# target_link_libraries(canlib_dump
#    can_interface_string
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
# )

# socketcan_bcm
add_executable(socketcan_bcm
  src/socketcan_bcm.cpp
)

target_link_libraries(socketcan_bcm
   can_interface_string
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

# canlib_bcm
# add_executable(canlib_bcm
#   src/canlib_bcm.cpp
# )

# target_link_libraries(canlib_bcm
#    can_interface_string
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
# )

# socketcan_interface_plugin
add_library(socketcan_interface_plugin
  src/socketcan_interface_plugin.cpp
)
target_link_libraries(socketcan_interface_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# canlib_interface_plugin
add_library(canlib_interface_plugin
  src/canlib_interface_plugin.cpp
)
target_link_libraries(canlib_interface_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    socketcan_bcm
    socketcan_dump
    socketcan_interface_plugin
#    canlib_bcm
#    canlib_dump
    canlib_interface_plugin
    can_interface_string
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  FILES
    socketcan_interface_plugin.xml
    canlib_interface_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(${PROJECT_NAME}-test_dummy_interface
    test/test_dummy_interface.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test_dummy_interface
    can_interface_string
    ${catkin_LIBRARIES}
  )
endif()
