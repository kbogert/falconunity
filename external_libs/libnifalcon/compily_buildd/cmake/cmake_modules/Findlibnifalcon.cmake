# Find libnifalcon 
# If found, will create the following values
#
#  LIBNIFALCON_FOUND - system has base libnifalcon c++ libraries
#  LIBNIFALCON_INCLUDE_DIRS - the libnifalcon c++ include directory
#  LIBNIFALCON_LIBRARIES - libnifalcon c++ libraries
#
#  LIBNIFALCON_CLI_BASE_FOUND - system has cli base utilities for falcon programs
#  LIBNIFALCON_CLI_BASE_LIBRARY - libraries required to use cli base utilities
#  LIBNIFALCON_DEVICE_BOOST_THREAD_FOUND - system has boost thread utilities for falcon programs
#  LIBNIFALCON_DEVICE_BOOST_THREAD_LIBRARY - libraries required to use boost thread device
#
# Copyright (c) 2008-2009 Kyle Machulis/Nonpolynomial Labs <kyle@nonpolynomial.com>
#

if (NOT libnifalcon_FIND_QUIETLY)
  MESSAGE(STATUS "Finding libnifalcon...")
endif (NOT libnifalcon_FIND_QUIETLY)
if(NOT LIBNIFALCON_SEARCH_DIR)
	   SET(LIBNIFALCON_SEARCH_DIR ".")
endif(NOT LIBNIFALCON_SEARCH_DIR)

if (LIBNIFALCON_LIBRARIES AND LIBNIFALCON_INCLUDE_DIRS)
  set(LIBNIFALCON_FOUND TRUE)
else (LIBNIFALCON_LIBRARIES AND LIBNIFALCON_INCLUDE_DIRS)
  find_path(LIBNIFALCON_INCLUDE_DIR
	NAMES
	falcon
	PATHS
    /usr/include/
    /usr/local/include/
    /opt/local/include/
    /sw/include/
	${LIBNIFALCON_SEARCH_DIR}/include/
	)
  
  find_library(LIBNIFALCON_LIBRARY
    NAMES
    nifalcon
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
	${LIBNIFALCON_SEARCH_DIR}/lib/
	)

  find_library(LIBNIFALCON_CLI_BASE_LIBRARY
    NAMES
    nifalcon_clibase
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
	${LIBNIFALCON_SEARCH_DIR}/lib/
	)

  find_library(LIBNIFALCON_DEVICE_BOOST_THREAD_LIBRARY
    NAMES
    nifalcon_device_boost_thread
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
	${LIBNIFALCON_SEARCH_DIR}/lib/
	)

  set(LIBNIFALCON_INCLUDE_DIRS
  	${LIBNIFALCON_INCLUDE_DIR}
	)
  
  set(LIBNIFALCON_LIBRARIES
    ${LIBNIFALCON_LIBRARY} 
	)

  if (LIBNIFALCON_INCLUDE_DIRS AND LIBNIFALCON_LIBRARIES)
    set(LIBNIFALCON_FOUND TRUE)
  endif (LIBNIFALCON_INCLUDE_DIRS AND LIBNIFALCON_LIBRARIES)

  if (LIBNIFALCON_CLI_BASE_LIBRARY)
    set(LIBNIFALCON_CLI_BASE_FOUND TRUE)
  endif (LIBNIFALCON_CLI_BASE_LIBRARY)

  if (LIBNIFALCON_DEVICE_BOOST_THREAD_LIBRARY)
    set(LIBNIFALCON_DEVICE_BOOST_THREAD_FOUND TRUE)
  endif (LIBNIFALCON_DEVICE_BOOST_THREAD_LIBRARY)

  if (LIBNIFALCON_FOUND)
    if (NOT libnifalcon_FIND_QUIETLY)
      message(STATUS "Found libnifalcon")
	  message(STATUS " - Includes: ${LIBNIFALCON_INCLUDE_DIRS}")
	  message(STATUS " - Libraries: ${LIBNIFALCON_LIBRARIES}")
    endif (NOT libnifalcon_FIND_QUIETLY)
  else (LIBNIFALCON_FOUND)
    if (libnifalcon_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find libnifalcon")
    endif (libnifalcon_FIND_REQUIRED)
  endif (LIBNIFALCON_FOUND)
  mark_as_advanced(LIBNIFALCON_INCLUDE_DIRS LIBNIFALCON_LIBRARIES)
endif (LIBNIFALCON_LIBRARIES AND LIBNIFALCON_INCLUDE_DIRS)

