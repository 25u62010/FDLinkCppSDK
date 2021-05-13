cmake_minimum_required(VERSION 3.0.2)
project(fdilink_ahrs_test)

add_compile_options(-std=c++11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(OpenCV 4.0 QUIET)
find_package(Eigen3 3.1.0 QUIET)
find_package(serial REQUIRED 
            PATHS /usr/local/share/serial/cmake/ #look here
            NO_DEFAULT_PATH)

include_directories(
    include 
    ${OPENCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${serial_INCLUDE_DIRS}
)

set( 
    LIBS
    ${OpenCV_LIBRARIES}
    ${EIGEN3_LIBS} 
    ${serial_LIBRARIES}    
)

add_executable(${PROJECT_NAME} src/test.cpp src/AhrsDriver.cpp src/crc_table.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBS})