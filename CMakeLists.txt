cmake_minimum_required(VERSION 2.8.3)
project(ydlidar_lcm)

add_definitions(-std=c++11) # Use C++11
#find_package(PkgConfig REQUIRED)
#pkg_search_module(LCM REQUIRED lcm)

#add_subdirectory(sdk)

set(LCM_INCLUDE_DIRS /usr/local/include)
list(APPEND LCM_LIBRARIES  /usr/local/lib/liblcm.so)

set(SDK_PATH "./sdk/")

FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/impl/unix/*.cpp"
  "${SDK_PATH}/src/*.cpp"
)


include_directories(
  ${LCM_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/lcmtypes
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/sdk/include
  ${PROJECT_SOURCE_DIR}/sdk/src
)

add_executable(ydlidar_lcm src/ydlidar_lcm.cpp  ${SDK_SRC})

target_link_libraries(ydlidar_lcm
   ${catkin_LIBRARIES}  ${LCM_LIBRARIES} -lpthread
 )


