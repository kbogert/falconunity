######################################################################################
# Parse Arguments Macro (for named argument building)
######################################################################################

#taken from http://www.cmake.org/Wiki/CMakeMacroParseArguments

MACRO(PARSE_ARGUMENTS prefix arg_names option_names)
  SET(DEFAULT_ARGS)
  FOREACH(arg_name ${arg_names})    
    SET(${prefix}_${arg_name})
  ENDFOREACH(arg_name)
  FOREACH(option ${option_names})
    SET(${prefix}_${option} FALSE)
  ENDFOREACH(option)

  SET(current_arg_name DEFAULT_ARGS)
  SET(current_arg_list)
  FOREACH(arg ${ARGN})            
    SET(larg_names ${arg_names})    
    LIST(FIND larg_names "${arg}" is_arg_name)                   
    IF (is_arg_name GREATER -1)
      SET(${prefix}_${current_arg_name} ${current_arg_list})
      SET(current_arg_name ${arg})
      SET(current_arg_list)
    ELSE (is_arg_name GREATER -1)
      SET(loption_names ${option_names})    
      LIST(FIND loption_names "${arg}" is_option)            
      IF (is_option GREATER -1)
		SET(${prefix}_${arg} TRUE)
      ELSE (is_option GREATER -1)
		LIST(APPEND current_arg_list ${arg})
      ENDIF (is_option GREATER -1)
    ENDIF (is_arg_name GREATER -1)
  ENDFOREACH(arg)
  SET(${prefix}_${current_arg_name} ${current_arg_list})
ENDMACRO(PARSE_ARGUMENTS)

######################################################################################
# Compile flag array building macro
######################################################################################

#taken from http://www.cmake.org/pipermail/cmake/2006-February/008334.html

MACRO(SET_COMPILE_FLAGS TARGET)
  SET(FLAGS)
  FOREACH(flag ${ARGN})
    SET(FLAGS "${FLAGS} ${flag}")
  ENDFOREACH(flag)
  SET_TARGET_PROPERTIES(${TARGET} PROPERTIES COMPILE_FLAGS "${FLAGS}")
ENDMACRO(SET_COMPILE_FLAGS)

######################################################################################
# Generalized library building function for all C++ libraries
######################################################################################

FUNCTION(BUILDSYS_BUILD_LIB)
  PARSE_ARGUMENTS(BUILDSYS_LIB
    "NAME;SOURCES;CXX_FLAGS;LINK_LIBS;LINK_FLAGS;DEPENDS;LIB_TYPES_OVERRIDE;SHOULD_INSTALL;VERSION"
    ""
    ${ARGN}
    )
  IF(BUILDSYS_LIB_LIB_TYPES_OVERRIDE)
	SET(BUILDSYS_LIB_TYPES_LIST ${BUILDSYS_LIB_LIB_TYPES_OVERRIDE})
  ELSE(BUILDSYS_LIB_LIB_TYPES_OVERRIDE)
	SET(BUILDSYS_LIB_TYPES_LIST ${BUILDSYS_LIB_TYPES})
  ENDIF(BUILDSYS_LIB_LIB_TYPES_OVERRIDE)

  LIST(REMOVE_DUPLICATES BUILDSYS_LIB_SOURCES)

  FOREACH(LIB_TYPE ${BUILDSYS_LIB_TYPES_LIST})
    SET(CURRENT_LIB ${BUILDSYS_LIB_NAME}_${LIB_TYPE})
    ADD_LIBRARY (${CURRENT_LIB} ${LIB_TYPE} ${BUILDSYS_LIB_SOURCES})
    LIST(APPEND LIB_DEPEND_LIST ${CURRENT_LIB})
    SET_TARGET_PROPERTIES (${CURRENT_LIB} PROPERTIES OUTPUT_NAME ${BUILDSYS_LIB_NAME})
    SET_TARGET_PROPERTIES (${CURRENT_LIB} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    SET_TARGET_PROPERTIES (${CURRENT_LIB} PROPERTIES SOVERSION ${BUILDSYS_LIB_VERSION})
    SET_TARGET_PROPERTIES (${CURRENT_LIB} PROPERTIES VERSION ${BUILDSYS_LIB_VERSION})

    #optional arguments
    IF(BUILDSYS_LIB_LINK_LIBS)
	  TARGET_LINK_LIBRARIES(${CURRENT_LIB} ${BUILDSYS_LIB_LINK_LIBS})
    ENDIF(BUILDSYS_LIB_LINK_LIBS)

    #cpp defines
    IF(BUILDSYS_LIB_CXX_FLAGS)
      SET_COMPILE_FLAGS(${CURRENT_LIB} ${BUILDSYS_LIB_CXX_FLAGS})
    ENDIF(BUILDSYS_LIB_CXX_FLAGS)

    IF(BUILDSYS_LIB_LINK_FLAGS)
      SET_TARGET_PROPERTIES(${CURRENT_LIB} PROPERTIES LINK_FLAGS ${BUILDSYS_LIB_LINK_FLAGS})
    ENDIF(BUILDSYS_LIB_LINK_FLAGS)

    #installation for non-windows platforms
    IF(BUILDSYS_LIB_SHOULD_INSTALL)
      INSTALL(TARGETS ${CURRENT_LIB} LIBRARY DESTINATION ${LIBRARY_INSTALL_DIR} ARCHIVE DESTINATION ${LIBRARY_INSTALL_DIR})
    ENDIF(BUILDSYS_LIB_SHOULD_INSTALL)

    #rewrite of install_name_dir in apple binaries
    IF(APPLE)
      SET_TARGET_PROPERTIES(${CURRENT_LIB} PROPERTIES INSTALL_NAME_DIR ${LIBRARY_INSTALL_DIR})
    ENDIF(APPLE)

    #We don't want our libraries built off in configuration type directories, and since
    #we take care of out of source configuration placement with the last command, we
    #don't need to worry about stomping on our own libs (i.e. only one type will be built
    #per project)
    IF(MSVC AND NOT NMAKE)
      SET_TARGET_PROPERTIES(${CURRENT_LIB} PROPERTIES PREFIX "../")
    ENDIF(MSVC AND NOT NMAKE)

    IF(BUILDSYS_LIB_DEPENDS)
      ADD_DEPENDENCIES(${CURRENT_LIB} ${BUILDSYS_LIB_DEPENDS})
    ENDIF(BUILDSYS_LIB_DEPENDS)
  ENDFOREACH(LIB_TYPE)
  SET(DEPEND_NAME "${BUILDSYS_LIB_NAME}_DEPEND")
  ADD_CUSTOM_TARGET(${DEPEND_NAME} DEPENDS ${LIB_DEPEND_LIST})
  
ENDFUNCTION(BUILDSYS_BUILD_LIB)

######################################################################################
# Generalized executable building function
######################################################################################

FUNCTION(BUILDSYS_BUILD_EXE)
  PARSE_ARGUMENTS(BUILDSYS_EXE
    "NAME;SOURCES;CXX_FLAGS;LINK_LIBS;LINK_FLAGS;DEPENDS;SHOULD_INSTALL"
    ""
    ${ARGN}
    )
  
  LIST(REMOVE_DUPLICATES BUILDSYS_EXE_SOURCES)

  ADD_EXECUTABLE(${BUILDSYS_EXE_NAME} ${BUILDSYS_EXE_SOURCES})
  SET_TARGET_PROPERTIES (${BUILDSYS_EXE_NAME} PROPERTIES OUTPUT_NAME ${BUILDSYS_EXE_NAME})

  IF(BUILDSYS_EXE_CXX_FLAGS)
    SET_COMPILE_FLAGS(${BUILDSYS_EXE_NAME} ${BUILDSYS_EXE_CXX_FLAGS})
  ENDIF(BUILDSYS_EXE_CXX_FLAGS)

  IF(BUILDSYS_EXE_LINK_FLAGS)
    SET_TARGET_PROPERTIES(${BUILDSYS_EXE_NAME} PROPERTIES LINK_FLAGS ${BUILDSYS_EXE_LINK_FLAGS})
  ENDIF(BUILDSYS_EXE_LINK_FLAGS)
  
  IF(BUILDSYS_EXE_LINK_LIBS)
    TARGET_LINK_LIBRARIES(${BUILDSYS_EXE_NAME} ${BUILDSYS_EXE_LINK_LIBS})
  ENDIF(BUILDSYS_EXE_LINK_LIBS)

  IF(BUILDSYS_EXE_SHOULD_INSTALL)
    INSTALL(TARGETS ${BUILDSYS_EXE_NAME} RUNTIME DESTINATION ${RUNTIME_INSTALL_DIR})
  ENDIF(BUILDSYS_EXE_SHOULD_INSTALL)

  IF(MSVC AND NOT NMAKE)
    SET_TARGET_PROPERTIES(${CURRENT_LIB} PROPERTIES PREFIX "../")
  ENDIF(MSVC AND NOT NMAKE)

  IF(BUILDSYS_EXE_DEPENDS)
    ADD_DEPENDENCIES(${BUILDSYS_EXE_NAME} ${BUILDSYS_EXE_DEPENDS})
  ENDIF(BUILDSYS_EXE_DEPENDS)
ENDFUNCTION(BUILDSYS_BUILD_EXE)

######################################################################################
# Make sure we aren't trying to do an in-source build
######################################################################################

#taken from http://www.mail-archive.com/cmake@cmake.org/msg14236.html

MACRO(MACRO_ENSURE_OUT_OF_SOURCE_BUILD)
  STRING(COMPARE EQUAL "${${PROJECT_NAME}_SOURCE_DIR}" "${${PROJECT_NAME}_BINARY_DIR}" insource)
  GET_FILENAME_COMPONENT(PARENTDIR ${${PROJECT_NAME}_SOURCE_DIR} PATH)
  STRING(COMPARE EQUAL "${${PROJECT_NAME}_SOURCE_DIR}" "${PARENTDIR}" insourcesubdir)
  IF(insource OR insourcesubdir)
    MESSAGE(FATAL_ERROR 
      "${PROJECT_NAME} requires an out of source build (make a build dir and call cmake from that.)\n"
      "A script (Makefile or python) should've been included in your build to generate this, check your project root directory.\n"
      "If you get this error from a sub-directory, make sure there is not a CMakeCache.txt in your project root directory."
      )
  ENDIF(insource OR insourcesubdir)
ENDMACRO(MACRO_ENSURE_OUT_OF_SOURCE_BUILD)

######################################################################################
# Create a library name that fits our platform
######################################################################################

MACRO(CREATE_LIBRARY_LINK_NAME LIBNAME)
  if(BUILD_STATIC AND NOT BUILD_SHARED)
    IF(NOT MSVC)
      SET(LIB_STATIC_PRE "lib")
      SET(LIB_STATIC_EXT ".a")
    ELSE(NOT MSVC)
      SET(LIB_STATIC_PRE "")
      SET(LIB_STATIC_EXT ".lib")
    ENDIF(NOT MSVC)
    SET(LIB_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH}/)
  ELSE(BUILD_STATIC AND NOT BUILD_SHARED)
    SET(LIB_STATIC_PRE)
    SET(LIB_STATIC_EXT)
    SET(LIB_OUTPUT_PATH)
  ENDIF(BUILD_STATIC AND NOT BUILD_SHARED)
  SET(lib${LIBNAME}_LIBRARY ${LIB_OUTPUT_PATH}${LIB_STATIC_PRE}${LIBNAME}${LIB_STATIC_EXT})

  
ENDMACRO(CREATE_LIBRARY_LINK_NAME)

######################################################################################
# Our own LAPACK finder
######################################################################################

#Cmake tries to find lapack/blas using fortran, which, while sane, is annoying. This
#macro bypasses that and assumes we've got lapack and blas set up in a certain way.

MACRO(FIND_PLATFORM_LAPACK)
  IF(APPLE AND ACCELERATE_FRAMEWORK)
    #On OS X, Accelerate ships with the platform
    SET(LAPACK_LIBRARIES ${ACCELERATE_LIBRARY})
  ELSEIF(MINGW)
    #On MinGW, we usually have gfortran, so use the regular find
    ENABLE_LANGUAGE(Fortran)
    SET(BLA_STATIC ON)
    FIND_PACKAGE(LAPACK REQUIRED)
    LIST(APPEND LAPACK_LIBRARIES gfortran)
  ELSEIF(MSVC)
    #On MSVC, we use a found blas/lapack
    #Gotten at http://www.stanford.edu/~vkl/code/libs.html
    SET(LAPACK_LIBRARIES blas_win32 lapack_win32)
  ELSE(APPLE AND ACCELERATE_FRAMEWORK)
    #*NIX. Set and pray.
    SET(LAPACK_LIBRARIES blas lapack)
  ENDIF(APPLE AND ACCELERATE_FRAMEWORK)
ENDMACRO(FIND_PLATFORM_LAPACK)

######################################################################################
# CPack Source Distro Setup
######################################################################################

MACRO(BUILDSYS_CPACK_INFO)
  PARSE_ARGUMENTS(BUILDSYS_CPACK
    "NAME;MAJOR_VERSION;MINOR_VERSION;BUILD_VERSION;VENDOR;DESCRIPTION"
    ""
    ${ARGN}
    )
  
  # CPack version numbers for release tarball name.
  SET(CPACK_PACKAGE_VERSION_MAJOR ${BUILDSYS_CPACK_MAJOR_VERSION})
  SET(CPACK_PACKAGE_VERSION_MINOR ${BUILDSYS_CPACK_MINOR_VERSION})
  SET(CPACK_PACKAGE_VERSION_PATCH ${BUILDSYS_CPACK_BUILD_VERSION})
  
  SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${BUILDSYS_CPACK_DESCRIPTION})
  SET(CPACK_PACKAGE_VENDOR ${BUILDSYS_CPACK_VENDOR})
  #We'll always have a description file handy as README
  SET(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README.txt)
  SET(CPACK_GENERATOR TGZ)
  SET(BUILDSYS_CPACK_VERSION ${BUILDSYS_CPACK_MAJOR_VERSION}.${BUILDSYS_CPACK_MINOR_VERSION}.${BUILDSYS_CPACK_BUILD_VERSION})
  SET(BUILDSYS_CPACK_NAME ${BUILDSYS_CPACK_NAME})

  BUILDSYS_CPACK_SOURCE_DISTRO()
ENDMACRO(BUILDSYS_CPACK_INFO)

MACRO(BUILDSYS_CPACK_SOURCE_DISTRO)
  set(
    CPACK_SOURCE_PACKAGE_FILE_NAME "${BUILDSYS_CPACK_NAME}-${BUILDSYS_CPACK_VERSION}-src"
    CACHE INTERNAL "${BUILDSYS_CPACK_NAME} Source Distribution"
    )
  set(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
  set(CPACK_SOURCE_IGNORE_FILES
    "~$"
    "^${PROJECT_SOURCE_DIR}/boneyard/"
    "^${PROJECT_SOURCE_DIR}/.git.*"
    "^${PROJECT_SOURCE_DIR}/build.*"
    )
ENDMACRO(BUILDSYS_CPACK_SOURCE_DISTRO)

