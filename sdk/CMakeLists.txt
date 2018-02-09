
add_definitions(-std=c++11) # Use C++11
include_directories(include)
include_directories(src)

IF (WIN32)
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.h"
  "src/impl/windows/*.cpp"
  "src/impl/windows/*.h"
)
	
ELSE()
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.h"
  "src/impl/unix/*.cpp"
  "src/impl/unix/*.h"
)

	
ENDIF()

add_subdirectory(samples)

add_library(ydlidar_driver SHARED ${SDK_SRC})
add_library(ydlidar_driver_static  STATIC ${SDK_SRC})

SET_TARGET_PROPERTIES(ydlidar_driver_static PROPERTIES OUTPUT_NAME "ydlidar_driver")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
IF (WIN32)
ELSE()
target_link_libraries(ydlidar_driver rt pthread)
target_link_libraries(ydlidar_driver_static rt pthread)
ENDIF()                    
