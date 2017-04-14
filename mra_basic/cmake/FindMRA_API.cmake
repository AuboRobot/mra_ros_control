# - Try to find libMRA_API
# Once done this will define
#
# libMRA_API_FOUND - system has libMRA_API
# libMRA_API_INCLUDE_DIRS - the libMRA_API include directories
# libMRA_API_LIBS - link these to use libMRA_API

find_path(libMRA_API_INCLUDE_DIR
	NAMES mra_api.h
	PATHS /usr/include/MRA_API
	      /usr/local/include/MRA_API
	      ${PROJECT_SOURCE_DIR}/include/MRA_API
	      $ENV{INCLUDE}
)


set(libMRA_API_INCLUDE_DIRS ${libMRA_API_INCLUDE_DIR})
message(STATUS ${libMRA_API_INCLUDE_DIR})
message(STATUS ${CMAKE_SYSTEM_PROCESSOR} "---" ${CMAKE_SIZEOF_VOID_P})
#Methord 1 : Automatically judge system digets by camke parameter "CMAKE_SIZEOF_VOID_P"
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(STATUS "System LONG_BIT is 64")
	find_library(libMRA_API_LIB
		NAMES CANAPI
		PATHS ${PROJECT_SOURCE_DIR}/lib/MRA_API64
	)
else()
        message(STATUS "System LONG_BIT is 32")
	find_library(libMRA_API_LIB
		NAMES CANAPI
		PATHS ${PROJECT_SOURCE_DIR}/lib/MRA_API32
	)
endif()

#Methord 2: user set the LONG_BIT parameter by hand in top CMakelist.txt
#if(${LONG_BIT} STREQUAL "32")
#        message(STATUS "System LONG_BIT is setted to 32")
#	find_library(libMRA_API_LIB
#		NAMES CANAPI
#		PATHS ${PROJECT_SOURCE_DIR}/lib/MRA_API32
#	)
#else()
#        message(STATUS "System LONG_BIT is setted to 64")
#	find_library(libMRA_API_LIB
#		NAMES CANAPI
#		PATHS ${PROJECT_SOURCE_DIR}/lib/MRA_API64
#	)
#endif()
	

set(libMRA_API_LIBS ${libMRA_API_LIB})

if(libMRA_API_INCLUDE_DIRS)
	message(STATUS "Found MRA_API include dir: ${libMRA_API_INCLUDE_DIRS}")
else(libMRA_API_INCLUDE_DIRS)
	message(STATUS "Could NOT find MRA_API headers.")
endif(libMRA_API_INCLUDE_DIRS)

if(libMRA_API_LIBS)
	message(STATUS "Found MRA_API library: ${libMRA_API_LIBS}")
else(libMRA_API_LIBS)
	message(STATUS "Could NOT find libMRA_API library.")
endif(libMRA_API_LIBS)

if(libMRA_API_INCLUDE_DIRS AND libMRA_API_LIBS)
	set(libMRA_API_FOUND TRUE)
else(libMRA_API_INCLUDE_DIRS AND libMRA_API_LIBS)
	set(libMRA_API_FOUND FALSE)
	message(FATAL_ERROR "Could not find MRA_API.")
endif(libMRA_API_INCLUDE_DIRS AND libMRA_API_LIBS)
