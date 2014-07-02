# this one is important
SET(CMAKE_SYSTEM_NAME Windows)
#this one not so much
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(CMAKE_C_COMPILER /usr/local/i386-mingw32msvc/bin/i386-mingw32msvc-gcc)
SET(CMAKE_CXX_COMPILER /usr/local/i386-mingw32msvc/bin/i386-mingw32msvc-g++)

#CMake will reset these variables unless we force them
#This will cause static library building to break (CMAKE_AR gets set to "")
SET(CMAKE_AR /usr/local/i386-mingw32msvc/bin/i386-mingw32msvc-ar CACHE FILENAME "AR Override" FORCE)
SET(CMAKE_RANLIB /usr/local/i386-mingw32msvc/bin/i386-mingw32msvc-ranlib CACHE FILENAME "RANLIB Override" FORCE)

# where is the target environment 
LIST(APPEND CMAKE_FIND_ROOT_PATH /usr/local/i386-mingw32msvc/)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
#SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
#SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

