include_directories(include ${catkin_INCLUDE_DIRS})

###########
# Library #
###########
# cs_add_library(${PROJECT_NAME}
#   rosbag_helper.cpp
# )

find_package(OpenCV REQUIRED)

##############
# EXECUTABLE #
##############
cs_add_executable(cmu_navigation_interface cmu_navigation_interface.cpp)
target_link_libraries(cmu_navigation_interface ${catkin_LIBRARIES} ${OpenCV_LIBS})

cs_add_executable(vis_navigation_interface vis_navigation_interface.cpp)
target_link_libraries(vis_navigation_interface ${catkin_LIBRARIES} ${OpenCV_LIBS})

########
# TEST #
########

