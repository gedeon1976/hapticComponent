cmake_minimum_required(VERSION 2.8)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${mt_INCLUDE_DIRS}) 
link_directories(${OpenCV_LIB_DIR} ${Boost_LIBRARY_DIRS})
  

set(HapticComponent_SRCS 
					InterfaceHaptic.hpp
					HapticAccess.h
					commonHapticComponent.h		
					haptic.h 
					haptic.cpp)


add_library(HapticComponent STATIC ${HapticComponent_SRCS})
set(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib )

# copy the interface header to the library
file(COPY	InterfaceHaptic.hpp 
			commonHapticComponent.h
			HapticAccess.h 
			haptic.h			
			DESTINATION ${CMAKE_HOME_DIRECTORY}/include)

message("output directory is: " ${LIBRARY_OUTPUT_PATH})

#add_executable(StereoCameraComponent ${StereoCameraComponent_SRCS})
#if (UNIX) 
#  target_link_libraries(StereoCameraComponent ${OpenCV_LIBS})
#endif (UNIX)
#if (WIN32)
#  target_link_libraries(StereoCameraComponent ${OpenCV_LIBS})
#endif (WIN32)