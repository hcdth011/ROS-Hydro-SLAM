# Install script for directory: /home/will/catkin_ws/src/libuvc

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/lib/libuvc.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libuvc.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libuvc.so"
         RPATH "")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libuvc.so")
FILE(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/will/catkin_ws/src/libuvc/build/libuvc.so")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/lib/libuvc.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libuvc.so")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libuvc.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/libuvc/libuvc.h;/usr/local/include/libuvc/libuvc_config.h")
FILE(INSTALL DESTINATION "/usr/local/include/libuvc" TYPE FILE FILES
    "/home/will/catkin_ws/src/libuvc/include/libuvc/libuvc.h"
    "/home/will/catkin_ws/src/libuvc/build/include/libuvc/libuvc_config.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/libuvc/libuvcConfig.cmake;/usr/local/lib/cmake/libuvc/libuvcConfigVersion.cmake")
FILE(INSTALL DESTINATION "/usr/local/lib/cmake/libuvc" TYPE FILE FILES
    "/home/will/catkin_ws/src/libuvc/build/CMakeFiles/libuvcConfig.cmake"
    "/home/will/catkin_ws/src/libuvc/build/libuvcConfigVersion.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/lib/cmake/libuvc/libuvcTargets.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}/usr/local/lib/cmake/libuvc/libuvcTargets.cmake"
         "/home/will/catkin_ws/src/libuvc/build/CMakeFiles/Export/_usr/local/lib/cmake/libuvc/libuvcTargets.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}/usr/local/lib/cmake/libuvc/libuvcTargets-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}/usr/local/lib/cmake/libuvc/libuvcTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/libuvc/libuvcTargets.cmake")
FILE(INSTALL DESTINATION "/usr/local/lib/cmake/libuvc" TYPE FILE FILES "/home/will/catkin_ws/src/libuvc/build/CMakeFiles/Export/_usr/local/lib/cmake/libuvc/libuvcTargets.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
     "/usr/local/lib/cmake/libuvc/libuvcTargets-release.cmake")
FILE(INSTALL DESTINATION "/usr/local/lib/cmake/libuvc" TYPE FILE FILES "/home/will/catkin_ws/src/libuvc/build/CMakeFiles/Export/_usr/local/lib/cmake/libuvc/libuvcTargets-release.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/will/catkin_ws/src/libuvc/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/will/catkin_ws/src/libuvc/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
