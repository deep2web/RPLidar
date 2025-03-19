cmake_minimum_required(VERSION 3.4)
project(pyrplidar_c1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 REQUIRED)

# Pfad zur RPLIDAR SDK - wir sind bereits im SDK-Verzeichnis
set(RPLIDAR_SDK_PATH ".")

# Quell-Dateien der SDK
file(GLOB RPLIDAR_SDK_SRC 
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

# Include-Pfade
include_directories(
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
)

# Erstellung des Python-Moduls
pybind11_add_module(pyrplidar_c1 rplidar_wrapper.cpp ${RPLIDAR_SDK_SRC})
