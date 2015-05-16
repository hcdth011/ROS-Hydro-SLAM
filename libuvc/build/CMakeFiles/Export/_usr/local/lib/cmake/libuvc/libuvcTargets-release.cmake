#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
SET(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "uvc" for configuration "Release"
SET_PROPERTY(TARGET uvc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
SET_TARGET_PROPERTIES(uvc PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "-ljpeg;/usr/lib/i386-linux-gnu/libusb-1.0.so"
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/libuvc.so"
  IMPORTED_SONAME_RELEASE "libuvc.so"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS uvc )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_uvc "/usr/local/lib/libuvc.so" )

# Loop over all imported files and verify that they actually exist
FOREACH(target ${_IMPORT_CHECK_TARGETS} )
  FOREACH(file ${_IMPORT_CHECK_FILES_FOR_${target}} )
    IF(NOT EXISTS "${file}" )
      MESSAGE(FATAL_ERROR "The imported target \"${target}\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    ENDIF()
  ENDFOREACH()
  UNSET(_IMPORT_CHECK_FILES_FOR_${target})
ENDFOREACH()
UNSET(_IMPORT_CHECK_TARGETS)

# Commands beyond this point should not need to know the version.
SET(CMAKE_IMPORT_FILE_VERSION)
