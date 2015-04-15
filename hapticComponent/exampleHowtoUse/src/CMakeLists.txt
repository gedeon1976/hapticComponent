cmake_minimum_required(VERSION 2.8)



# include directories
if (UNIX)
  include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${3DPOINTER_INCLUDE_DIR}) 
  
else (UNIX)
  include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${3DPOINTER_INCLUDE_DIR}) 
 
endif (UNIX)

# set source code
set(testHapticComponent_SRCS 
					testHapticComponent.cpp)

# generate executable file
add_executable(testHapticComponent ${testHapticComponent_SRCS})

if (UNIX) 
  target_link_libraries(testHapticComponent ${OpenCV_LIBS} ${Boost_LIBRARIES} ${3DPOINTER_LIBRARY})
endif (UNIX)
if (WIN32)
  target_link_libraries(testHapticComponent ${OpenCV_LIBS} ${Boost_LIBRARIES} ${3DPOINTER_LIBRARY})
endif (WIN32)