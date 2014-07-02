######################################################################################
# Add subdirectories
######################################################################################

SET(BUILDSYS_SCRIPT_DIR ${BUILDSYS_CMAKE_DIR}/scripts)
LIST(APPEND CMAKE_MODULE_PATH ${BUILDSYS_CMAKE_DIR}/cmake_modules)

INCLUDE(${BUILDSYS_CMAKE_DIR}/CMakeOptions.cmake)
INCLUDE(${BUILDSYS_CMAKE_DIR}/CMakeFunctions.cmake)

######################################################################################
# Initialize Build Function
######################################################################################

MACRO(INITIALIZE_BUILD)

  MACRO_ENSURE_OUT_OF_SOURCE_BUILD()

  IF(NOT BUILDSYS_BUILD_PLATFORM)
    IF(BUILDSYS_REQUIRE_VARS)
      MESSAGE(FATAL_ERROR "-- BUILDSYS_BUILD_PLATFORM variable not set.")
    ELSE(BUILDSYS_REQUIRE_VARS)
      MESSAGE("-- BUILDSYS_BUILD_PLATFORM variable not set. (Just a warning, you can probably ignore this.)")
    ENDIF(BUILDSYS_REQUIRE_VARS)
  ELSE(NOT BUILDSYS_BUILD_PLATFORM)
    MESSAGE(STATUS "Build Platform: ${BUILDSYS_BUILD_PLATFORM}")
    SET(PREFIX_DIR ${PROJECT_SOURCE_DIR}/../library/usr_${BUILDSYS_BUILD_PLATFORM})
    SET(BUILDSYS_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/../library/usr_${BUILDSYS_BUILD_PLATFORM})  
  ENDIF(NOT BUILDSYS_BUILD_PLATFORM)

  IF(NOT BUILDSYS_BUILD_DIR)
    IF(BUILDSYS_REQUIRE_VARS)
      MESSAGE(FATAL_ERROR "-- BUILDSYS_BUILD_DIR variable not set.")
    ELSE(BUILDSYS_REQUIRE_VARS)
      MESSAGE("-- BUILDSYS_BUILD_DIR variable not set. (Just a warning, you can probably ignore this.)")
    ENDIF(BUILDSYS_REQUIRE_VARS)
  ELSE(NOT BUILDSYS_BUILD_DIR)
    MESSAGE(STATUS "Build Directory: ${BUILDSYS_BUILD_DIR}")
  ENDIF(NOT BUILDSYS_BUILD_DIR)

  IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
    MESSAGE(STATUS "No build type specified, using default: ${CMAKE_BUILD_TYPE}")
  ENDIF(NOT CMAKE_BUILD_TYPE)

  #Global Options that should be included with all projects
  OPTION_FORCE_32_BIT(OFF)

  SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib)

  IF(NOT CMAKE_CROSSCOMPILING AND BUILDSYS_LIBRARY_PATH)
    SET(CMAKE_FIND_ROOT_PATH ${BUILDSYS_LIBRARY_PATH})
  ELSEIF(BUILDSYS_LIBRARY_PATH)
    LIST(INSERT CMAKE_FIND_ROOT_PATH 0 ${BUILDSYS_LIBRARY_PATH})
  ENDIF(NOT CMAKE_CROSSCOMPILING AND BUILDSYS_LIBRARY_PATH)

  #Always assume we want to build threadsafe mingw binaries
  IF(MINGW)
    LIST(APPEND BUILDSYS_GLOBAL_DEFINES -mthreads)
    SET(CMAKE_LINK_FLAGS "${CMAKE_LINK_FLAGS} -mthreads")
  ENDIF(MINGW)

  #defines we always need on gcc compilers
  IF(CMAKE_COMPILER_IS_GNUCXX)
    LIST(APPEND BUILDSYS_GLOBAL_DEFINES
      -DREENTRANT
      -D_REENTRANT
      -D_THREAD_SAFE
      -D_FILE_OFFSET_BITS=64
      -D_LARGEFILE_SOURCE
      )
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)

  IF(NOT MINGW)
    LIST(APPEND BUILDSYS_GLOBAL_DEFINES -D__STDC_LIMIT_MACROS)
  ENDIF(NOT MINGW)

  FOREACH(DEFINE ${BUILDSYS_GLOBAL_DEFINES})
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEFINE}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${DEFINE}")
  ENDFOREACH(DEFINE ${BUILDSYS_GLOBAL_DEFINES})	

  #taken from OpenSceneGraph CMake.
  #Handy visual studio functions
  #Assuming /MP to always be on though
  IF(MSVC)

    #Check to see if we're using nmake. If so, set the NMAKE variable
    IF(CMAKE_MAKE_PROGRAM STREQUAL "nmake")
      SET(NMAKE 1)
    ENDIF(CMAKE_MAKE_PROGRAM STREQUAL "nmake")

    # This option is to enable the /MP switch for Visual Studio 2005 and above compilers
    OPTION(WIN32_USE_MP "Set to ON to build with the /MP multiprocessor compile option (Visual Studio 2005 and above)." ON)
    IF(WIN32_USE_MP)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    ENDIF(WIN32_USE_MP)
    
    # turn off various warnings
    FOREACH(warning 4244 4251 4267 4274 4275 4290 4786 4305 4996)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd${warning}")
    ENDFOREACH(warning)
    
    # More MSVC specific compilation flags
    ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
    ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

    #Assume we always want NOMINMAX defined, and lean and mean,
    #and no winsock1. Tends to throw redefinition warnings, but eh.
    ADD_DEFINITIONS(-DNOMINMAX -DWIN32_LEAN_AND_MEAN -D_WINSOCKAPI_)

    #Only generate the project configuration we request
    SET(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE})
    MESSAGE(STATUS "CONFIG TYPES ${CMAKE_CONFIGURATION_TYPES}")

    #HACK FIX for runtime library linking. Never link against static runtime.
    #If you want to compile with static runtimes, you fix it. :3
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:\"LIBCMT.lib\" /NODEFAULTLIB:\"LIBCMTD.lib\" /NODEFAULTLIB:\"LIBC.lib\"")
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:\"LIBCMT.lib\" /NODEFAULTLIB:\"LIBCMTD.lib\" /NODEFAULTLIB:\"LIBC.lib\"")
  ENDIF(MSVC)

  IF(NOT PREFIX_DIR)
    IF(NOT WIN32)
      SET(PREFIX_DIR /usr/local)
    ELSE(NOT WIN32)
      SET(PREFIX_DIR "c:/Program Files/${PROJECT_NAME}")
    ENDIF(NOT WIN32)
  ENDIF(NOT PREFIX_DIR)
  IF(NOT INCLUDE_INSTALL_DIR)
    SET(INCLUDE_INSTALL_DIR ${PREFIX_DIR}/include)
  ENDIF(NOT INCLUDE_INSTALL_DIR)
  IF(NOT LIBRARY_INSTALL_DIR)
    SET(LIBRARY_INSTALL_DIR ${PREFIX_DIR}/lib)
  ENDIF(NOT LIBRARY_INSTALL_DIR)
  IF(NOT RUNTIME_INSTALL_DIR)
    SET(RUNTIME_INSTALL_DIR ${PREFIX_DIR}/bin)
  ENDIF(NOT RUNTIME_INSTALL_DIR)

  MESSAGE(STATUS "Install Directory Prefix: ${PREFIX_DIR}")
  MESSAGE(STATUS "Include Install Directory: ${INCLUDE_INSTALL_DIR}")
  MESSAGE(STATUS "Library Install Directory: ${LIBRARY_INSTALL_DIR}")
  MESSAGE(STATUS "Runtime Install Directory: ${RUNTIME_INSTALL_DIR}")
  
  IF(USE_DISTCC)
    #If the distccdir doesn't exist, things can fail in weird places during generation. 
    #So we run the host script during generation to make sure it's there.
    IF(NOT EXISTS "${BUILDSYS_CMAKE_DIR}/../distcc/distccdir")
      EXECUTE_PROCESS(COMMAND "${BUILDSYS_CMAKE_DIR}/../distcc/scripts/distcc_hosts_check.py")
    ENDIF(NOT EXISTS "${BUILDSYS_CMAKE_DIR}/../distcc/distccdir")

    # Every time we do a make, check for directory existence
    ADD_CUSTOM_TARGET(
      DISTCC_HOSTS_CHECK
      ALL
      COMMAND "${BUILDSYS_CMAKE_DIR}/../distcc/scripts/distcc_hosts_check.py"
      )
  ENDIF(USE_DISTCC)
ENDMACRO(INITIALIZE_BUILD)