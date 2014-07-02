######################################################################################
# CCache Option
######################################################################################

# MACRO(OPTION_USE_CCACHE DEFAULT)
#   OPTION(USE_CCACHE "Use ccache compiler caching system if available" ${DEFAULT})

#   IF(USE_CCACHE)
# 	SET(CMAKE_C_COMPILER "ccache gcc")
# 	SET(CMAKE_CXX_COMPILER "ccache g++")
# 	MESSAGE(STATUS "Turning on ccache for ${CMAKE_PROJECT_NAME}")
#   ELSE(USE_CCACHE)
# 	MESSAGE(STATUS "NOT using ccache ${CMAKE_PROJECT_NAME}")
#   ENDIF(USE_CCACHE)
# ENDMACRO(OPTION_USE_CCACHE)

######################################################################################
# Library Build Type Options
######################################################################################

MACRO(OPTION_LIBRARY_BUILD_STATIC DEFAULT)
  OPTION(BUILD_STATIC "Build static libraries" ${DEFAULT})

  IF(BUILD_STATIC)
	LIST(APPEND BUILDSYS_LIB_TYPES STATIC)
	MESSAGE(STATUS "Building Static Libraries for ${CMAKE_PROJECT_NAME}")
  ELSE(BUILD_STATIC)
  	MESSAGE(STATUS "NOT Building Static Libraries for ${CMAKE_PROJECT_NAME}")
  ENDIF(BUILD_STATIC)
ENDMACRO(OPTION_LIBRARY_BUILD_STATIC)

MACRO(OPTION_LIBRARY_BUILD_SHARED DEFAULT)
  OPTION(BUILD_SHARED "Build shared libraries" ${DEFAULT})

  IF(BUILD_SHARED)
	LIST(APPEND BUILDSYS_LIB_TYPES SHARED)
	MESSAGE(STATUS "Building Shared Libraries for ${CMAKE_PROJECT_NAME}")
  ELSE(BUILD_SHARED)
  	MESSAGE(STATUS "NOT Building Shared Libraries for ${CMAKE_PROJECT_NAME}")
  ENDIF(BUILD_SHARED)
ENDMACRO(OPTION_LIBRARY_BUILD_SHARED)

######################################################################################
# RPATH Relink Options
######################################################################################

MACRO(OPTION_BUILD_RPATH DEFAULT)
  OPTION(SET_BUILD_RPATH "Set the build RPATH to local directories, relink to install directories at install time" ${DEFAULT})

  IF(SET_BUILD_RPATH)
  	MESSAGE(STATUS "Setting build RPATH for ${CMAKE_PROJECT_NAME}")
	# use, i.e. don't skip the full RPATH for the build tree
	SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  
	# when building, don't use the install RPATH already
	# (but later on when installing)
	SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  
	# the RPATH to be used when installing
	SET(CMAKE_INSTALL_RPATH "${LIBRARY_INSTALL_DIR}")
  
	# add the automatically determined parts of the RPATH
	# which point to directories outside the build tree to the install RPATH
	SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  ELSE(SET_BUILD_RPATH)
    MESSAGE(STATUS "NOT Setting build RPATH for ${CMAKE_PROJECT_NAME}")
  ENDIF(SET_BUILD_RPATH)
ENDMACRO(OPTION_BUILD_RPATH)

######################################################################################
# Create software version code file
######################################################################################

MACRO(OPTION_CREATE_VERSION_FILE DEFAULT OUTPUT_FILES)
  OPTION(CREATE_VERSION_FILE "Creates a version.cc file using the setlocalversion script" ${DEFAULT})
  IF(CREATE_VERSION_FILE)
	MESSAGE(STATUS "Generating git information for ${CMAKE_PROJECT_NAME}")	
	FOREACH(VERSION_FILE ${OUTPUT_FILES})
	  MESSAGE(STATUS "- Generating to ${VERSION_FILE}")	
      SET(COMMAND_LIST "python" "${BUILDSYS_CMAKE_DIR}/../python/get_version.py" "-f" "${VERSION_FILE}" "-d" "${CMAKE_SOURCE_DIR}")
 	  EXECUTE_PROCESS(COMMAND ${COMMAND_LIST})
	ENDFOREACH(VERSION_FILE ${OUTPUT_FILES})
  ELSE(CREATE_VERSION_FILE)
	MESSAGE(STATUS "NOT generating git information for ${CMAKE_PROJECT_NAME}")	
  ENDIF(CREATE_VERSION_FILE)
ENDMACRO(OPTION_CREATE_VERSION_FILE)

######################################################################################
# Create software version code file
######################################################################################

MACRO(OPTION_CREATE_APP_KEYS_FILE DEFAULT OUTPUT_FILES)
  OPTION(CREATE_APP_KEYS_FILE "Creates a app_keys.cc file using the build_keys script" ${DEFAULT})
  IF(CREATE_APP_KEYS_FILE)
	MESSAGE(STATUS "Generating git information for ${CMAKE_PROJECT_NAME}")	
	FOREACH(APP_KEYS_FILE ${OUTPUT_FILES})
 	  EXECUTE_PROCESS(COMMAND "${BUILDSYS_SCRIPT_DIR}/build_keys.py" OUTPUT_FILE ${APP_KEYS_FILE})
	  MESSAGE(STATUS "- Generating to ${VERSION_FILE}")	
	ENDFOREACH(APP_KEYS_FILE ${OUTPUT_FILES})
  ELSE(CREATE_APP_KEYS_FILE)
	MESSAGE(STATUS "NOT generating git information for ${CMAKE_PROJECT_NAME}")	
  ENDIF(CREATE_APP_KEYS_FILE)
ENDMACRO(OPTION_CREATE_APP_KEYS_FILE)

######################################################################################
# Fast Math Option
######################################################################################

MACRO(OPTION_FAST_MATH DEFAULT)
  IF(CMAKE_COMPILER_IS_GNUCXX)
	OPTION(FAST_MATH "Use -ffast-math for GCC 4.0" ${DEFAULT})

	IF(FAST_MATH)
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffast-math")   
	  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")   
	  MESSAGE(STATUS "Turning on -ffast-math for ${CMAKE_PROJECT_NAME}")
	ELSE(FAST_MATH)
	  MESSAGE(STATUS "NOT Turning on -ffast-math for ${CMAKE_PROJECT_NAME}")
	ENDIF(FAST_MATH)
  ELSE(CMAKE_COMPILER_IS_GNUCXX)
	MESSAGE(STATUS "Fast math NOT AVAILABLE - Not a GNU compiler")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDMACRO(OPTION_FAST_MATH)

######################################################################################
# Turn on GProf based profiling 
######################################################################################

MACRO(OPTION_GPROF DEFAULT)
  IF(CMAKE_COMPILER_IS_GNUCXX)
	OPTION(ENABLE_GPROF "Compile using -g -pg for gprof output" ${DEFAULT})
	IF(ENABLE_GPROF)
	  MESSAGE(STATUS "Using gprof output for ${CMAKE_PROJECT_NAME}")
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -pg")
	  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -pg")
	  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g -pg")
	  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g -pg")
	ELSE(ENABLE_GPROF)
	  MESSAGE(STATUS "NOT using gprof output for ${CMAKE_PROJECT_NAME}")
	ENDIF(ENABLE_GPROF)
  ELSE(CMAKE_COMPILER_IS_GNUCXX)
	MESSAGE(STATUS "gprof generation NOT AVAILABLE - Not a GNU compiler")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDMACRO(OPTION_GPROF)

######################################################################################
# Platform specific optimizations
######################################################################################

MACRO(OPTION_ARCH_OPTS DEFAULT)
  IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
	MESSAGE(STATUS "Architecture Optimizations NOT AVAILABLE - Not a 32 bit system")
  ELSE(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
	IF(CMAKE_COMPILER_IS_GNUCXX)
	  IF(NOT CMAKE_CROSSCOMPILING)
		OPTION(ARCH_OPTS "Find and use architecture optimizations" ${DEFAULT})
		IF(ARCH_OPTS)
 		  EXECUTE_PROCESS(COMMAND "${BUILDSYS_SCRIPT_DIR}/gcccpuopt.sh" RESULT_VARIABLE CPU_RESULT OUTPUT_VARIABLE CPU_OPT ERROR_VARIABLE CPU_ERR OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
		  MESSAGE(STATUS "Using Processor optimizations for ${CMAKE_PROJECT_NAME}: ${CPU_OPT}")
		  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_OPT}")
		  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPU_OPT}")
		ELSE(ARCH_OPTS)
		  MESSAGE(STATUS "NOT Using Processor optimizations for ${CMAKE_PROJECT_NAME}")
		ENDIF(ARCH_OPTS)
	  ELSE(NOT CMAKE_CROSSCOMPILING)
		MESSAGE(STATUS "NOT Using Processor optimizations DUE TO CROSS COMPILING for ${CMAKE_PROJECT_NAME}")
	  ENDIF(NOT CMAKE_CROSSCOMPILING)
	ELSE(CMAKE_COMPILER_IS_GNUCXX)
	  MESSAGE(STATUS "Architecture Optimizations NOT AVAILABLE - Not a GNU compiler")
	ENDIF(CMAKE_COMPILER_IS_GNUCXX)
  ENDIF(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  
ENDMACRO(OPTION_ARCH_OPTS)

######################################################################################
# Turn on "extra" compiler warnings (SPAMMY WITH BOOST)
######################################################################################

MACRO(OPTION_EXTRA_COMPILER_WARNINGS DEFAULT)
  IF(CMAKE_COMPILER_IS_GNUCXX)
	OPTION(EXTRA_COMPILER_WARNINGS "Turn on -Wextra for gcc" ${DEFAULT})
	IF(EXTRA_COMPILER_WARNINGS)
	  MESSAGE(STATUS "Turning on extra c/c++ warnings for ${CMAKE_PROJECT_NAME}")
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra")
	  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
	ELSE(EXTRA_COMPILER_WARNINGS)
	  MESSAGE(STATUS "NOT turning on extra c/c++ warnings for ${CMAKE_PROJECT_NAME}")
	ENDIF(EXTRA_COMPILER_WARNINGS)
  ELSE(CMAKE_COMPILER_IS_GNUCXX)
	MESSAGE(STATUS "Extra compiler warnings NOT AVAILABLE - Not a GNU compiler")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDMACRO(OPTION_EXTRA_COMPILER_WARNINGS )

######################################################################################
# Turn on effective C++ compiler warnings
######################################################################################

MACRO(OPTION_EFFCXX_COMPILER_WARNINGS DEFAULT)
  IF(CMAKE_COMPILER_IS_GNUCXX)
	OPTION(EFFCXX_COMPILER_WARNINGS "Turn on -Weffc++ (effective c++ warnings) for gcc" ${DEFAULT})
	IF(EFFCXX_COMPILER_WARNINGS)
	  MESSAGE(STATUS "Turning on Effective c++ warnings for ${CMAKE_PROJECT_NAME}")
	  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weffc++")
	ELSE(EFFCXX_COMPILER_WARNINGS)
	  MESSAGE(STATUS "NOT turning on Effective c++ warnings for ${CMAKE_PROJECT_NAME}")
	ENDIF(EFFCXX_COMPILER_WARNINGS)
  ELSE(CMAKE_COMPILER_IS_GNUCXX)
	MESSAGE(STATUS "Effective C++ compiler warnings NOT AVAILABLE - Not a GNU compiler")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDMACRO(OPTION_EFFCXX_COMPILER_WARNINGS)

######################################################################################
# Return type compiler warnings
######################################################################################

MACRO(OPTION_RETURN_TYPE_COMPILER_WARNINGS DEFAULT)
  IF(CMAKE_COMPILER_IS_GNUCXX)
	OPTION(RETURN_TYPE_COMPILER_WARNINGS "Turn on -Wreturn-type for gcc" ${DEFAULT})
	IF(RETURN_TYPE_COMPILER_WARNINGS)
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wreturn-type")
	  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type")
	  MESSAGE(STATUS "Turning on return type warnings for ${CMAKE_PROJECT_NAME}")
	ELSE(RETURN_TYPE_COMPILER_WARNINGS)
	  MESSAGE(STATUS "NOT turning on return type warnings for ${CMAKE_PROJECT_NAME}")
	ENDIF(RETURN_TYPE_COMPILER_WARNINGS)
  ELSE(CMAKE_COMPILER_IS_GNUCXX)
	MESSAGE(STATUS "Return type warnings NOT AVAILABLE - Not a GNU compiler")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDMACRO(OPTION_RETURN_TYPE_COMPILER_WARNINGS)

######################################################################################
# Look for accelerate, if found, add proper includes
######################################################################################

MACRO(OPTION_ACCELERATE_FRAMEWORK DEFAULT)
  IF(APPLE)
	OPTION(ACCELERATE_FRAMEWORK "Use Accelerate Framework for Math (Adds -D_ACCELERATE_ for compiling and Accelerate Framework linking)" ${DEFAULT})
	IF(ACCELERATE_FRAMEWORK)
	  FIND_LIBRARY(ACCELERATE_LIBRARY Accelerate REQUIRED)
	  SET(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -D_ACCELERATE_")
	  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_ACCELERATE_")
	  MESSAGE(STATUS "Turning on Accelerate Framework for ${CMAKE_PROJECT_NAME}")
	ELSE(ACCELERATE_FRAMEWORK)
	  MESSAGE(STATUS "NOT turning on Accelerate Framework for ${CMAKE_PROJECT_NAME}")
	ENDIF(ACCELERATE_FRAMEWORK)
  ELSE(APPLE)
	MESSAGE(STATUS "Accelerate Framework NOT AVAILABLE - Not compiling for OS X")
  ENDIF(APPLE)
ENDMACRO(OPTION_ACCELERATE_FRAMEWORK)

######################################################################################
# Force 32-bit, regardless of the platform we're on
######################################################################################

MACRO(OPTION_FORCE_32_BIT DEFAULT)
  IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
	IF(CMAKE_COMPILER_IS_GNUCXX)
	  OPTION(FORCE_32_BIT "Force compiler to use -m32 when compiling" ${DEFAULT})
	  IF(FORCE_32_BIT)
		MESSAGE(STATUS "Forcing 32-bit on 64-bit platform (using -m32)")
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
		SET(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -m32")
		SET(CMAKE_LINK_FLAGS "${CMAKE_CXX_FLAGS} -m32")
	  ELSE(FORCE_32_BIT)
		MESSAGE(STATUS "Not forcing 32-bit on 64-bit platform")
	  ENDIF(FORCE_32_BIT)
	ELSE(CMAKE_COMPILER_IS_GNUCXX)
	  MESSAGE(STATUS "Force 32 bit NOT AVAILABLE - Not using gnu compiler")
	ENDIF(CMAKE_COMPILER_IS_GNUCXX)
  ELSE({CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
	MESSAGE(STATUS "Force 32 bit NOT AVAILABLE - Already on a 32 bit platform")
  ENDIF(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
ENDMACRO(OPTION_FORCE_32_BIT)
