cmake_minimum_required(VERSION 3.0.2)
project(agribot_path_follower)

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS 
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(path_follower src/path_follower.cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mark_follower src/mark_follower.cpp)
target_link_libraries(mark_follower ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(sprayer src/sprayer.cpp)
target_link_libraries(sprayer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(seeder src/seeder.cpp)
target_link_libraries(seeder ${catkin_LIBRARIES})