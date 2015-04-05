cmake_minimum_required(VERSION 2.8.12)

project(stereo_tracker)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("./StereoTracker/")

file(GLOB st_SRC
    "StereoTracker/*.h"
    "StereoTracker/*.cpp"
)

add_executable(stereo_tracker ${st_SRC})
target_link_libraries(stereo_tracker ${OpenCV_LIBS})

