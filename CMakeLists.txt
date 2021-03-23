cmake_minimum_required(VERSION 2.8.3)
project(visual_servoing_events)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  dvs_msgs
  message_generation
)

find_package(OpenCV REQUIRED)


catkin_package(
INCLUDE_DIRS LIBRARIES rosopencv_t CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs CATKIN_DEPENDS message_runtime std_msgs )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB SOURCE_FILES *.hpp *.cpp *.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_executable(visual_servoing_events_node src/main.cpp ${SOURCE_FILES})
add_executable(CheckDetection_node src/CheckDetection.cpp ${SOURCE_FILES})

target_link_libraries(visual_servoing_events_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(CheckDetection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})










