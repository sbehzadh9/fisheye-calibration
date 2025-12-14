cmake_minimum_required(VERSION 3.14)
project(FisheyeCalibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Fetch nlohmann_json
include(FetchContent)
FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
)
FetchContent_MakeAvailable(json)

add_executable(fisheye_calib src/main.cpp)

target_link_libraries(fisheye_calib PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

target_include_directories(fisheye_calib PRIVATE ${OpenCV_INCLUDE_DIRS})
