include_directories(include ${catkin_INCLUDE_DIRS})

###########
# Library #
###########
# cs_add_library(${PROJECT_NAME}
#   rosbag_helper.cpp
# )

##############
# EXECUTABLE #
##############
cs_add_executable(cmu_navigation_interface cmu_navigation_interface.cpp)
target_link_libraries(cmu_navigation_interface ${catkin_LIBRARIES})

cs_add_executable(vis_navigation_interface vis_navigation_interface.cpp)
target_link_libraries(vis_navigation_interface ${catkin_LIBRARIES})

########
# TEST #
########

