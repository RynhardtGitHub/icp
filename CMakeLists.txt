cmake_minimum_required(VERSION 3.10)

# Configure project, executable, source, and include directories
set(PROJECT_NAME ICP_Example)
set(EXECUTABLE_NAME icp_example)
set(SOURCE_DIR src)
set(INCLUDE_DIR inc)

project(${PROJECT_NAME} CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the header directory
include_directories(${INCLUDE_DIR})

# Add the executable and specify source files
add_executable(${EXECUTABLE_NAME} ${SOURCE_DIR}/main.cpp ${SOURCE_DIR}/ICP.cpp)

# Link with the math library
target_link_libraries(${EXECUTABLE_NAME} m)

# Set the build type to Debug for debugging symbols
set(CMAKE_BUILD_TYPE Debug)
