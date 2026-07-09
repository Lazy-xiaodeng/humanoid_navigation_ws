#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Open3D::Open3D" for configuration "Release"
set_property(TARGET Open3D::Open3D APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Open3D::Open3D PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libOpen3D.so"
  IMPORTED_SONAME_RELEASE "libOpen3D.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS Open3D::Open3D )
list(APPEND _IMPORT_CHECK_FILES_FOR_Open3D::Open3D "${_IMPORT_PREFIX}/lib/libOpen3D.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
