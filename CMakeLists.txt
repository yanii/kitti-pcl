cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(KTTIPCL)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH  ${PROJECT_BINARY_DIR}/lib)

#Note: Eclipse automatically picks up include paths with this on!
SET(CMAKE_VERBOSE_MAKEFILE ON)

##### Compiler Flags #####
IF(CMAKE_COMPILER_IS_GNUCXX)
	IF(DEBUG)
		SET(CMAKE_CXX_FLAGS "-g -O0 -fPIC -Wall ${OpenMP_CXX_FLAGS}  -DJSON_SAFE -DJSON_DEBUG" )
		SET(CMAKE_C_FLAGS "-g -O0 -fPIC -Wall ${OpenMP_C_FLAGS}  -DJSON_SAFE -DJSON_DEBUG" )
	ELSE(DEBUG)
		SET(CMAKE_CXX_FLAGS "-g -O3 -fPIC -Wall -fexpensive-optimizations ${OpenMP_CXX_FLAGS}" )
		SET(CMAKE_C_FLAGS "-g -O3 -fPIC -Wall -fexpensive-optimizations ${OpenMP_C_FLAGS}" )
	ENDIF(DEBUG)
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

##### Package Dependencies #####
include(UseDoxygen OPTIONAL)

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CUDA)

MESSAGE("PCL information:") 
MESSAGE("  PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}") 
MESSAGE("  PCL_LIBRARIES: ${PCL_LIBRARIES}") 
MESSAGE("  PCL_DEFINITIONS: ${PCL_DEFINITIONS}") 

set(BOOST_LIBS program_options serialization random system)
FIND_PACKAGE(Boost 1.48.0 COMPONENTS ${BOOST_LIBS} REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
MESSAGE("Boost information:") 
MESSAGE("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
MESSAGE("  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
MESSAGE("  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}") 

INCLUDE_DIRECTORIES(include)

##### Subdirectories #####
add_subdirectory(cmake)
add_subdirectory(src)
