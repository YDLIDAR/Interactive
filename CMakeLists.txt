cmake_minimum_required(VERSION 2.8)
#add_definitions(-std=c++11) # Use C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(include)
include_directories(src)

IF (WIN32)
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/windows/*.cpp"
  "src/impl/windows/*.h"
)
	
ELSE()
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/unix/*.cpp"
  "src/impl/unix/*.h"
)

	
ENDIF()

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
add_subdirectory(samples)
add_library(ydlidar_driver SHARED ${SDK_SRC})
add_library(ydlidar_driver_static STATIC ${SDK_SRC})

SET_TARGET_PROPERTIES(ydlidar_driver_static PROPERTIES OUTPUT_NAME "ydlidar_driver")
SET_TARGET_PROPERTIES(ydlidar_driver PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(ydlidar_driver_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
IF (WIN32)
target_link_libraries(ydlidar_driver ws2_32 setupapi)
target_link_libraries(ydlidar_driver_static ws2_32 setupapi)
ELSE()
target_link_libraries(ydlidar_driver rt pthread)
target_link_libraries(ydlidar_driver_static  rt pthread)
ENDIF()

include_directories( ${CMAKE_BINARY_DIR} )

INSTALL(TARGETS ydlidar_driver ydlidar_driver_static
       RUNTIME DESTINATION bin
       LIBRARY DESTINATION lib
       ARCHIVE DESTINATION lib
)

install(FILES include/v8stdint.h
    include/ChannelDevice.h
    include/ydlidar_driver.h
    include/ydlidar_protocol.h
    include/utils.h
    include/CYdlidar.h
    include/ETLidarDriver.h
    include/etlidar_protocol.h
    include/locker.h
    include/thread.h DESTINATION include)
