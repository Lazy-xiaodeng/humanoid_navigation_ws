#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Open3D::open3d_tf_ops" for configuration "Release"
set_property(TARGET Open3D::open3d_tf_ops APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Open3D::open3d_tf_ops PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "Open3D::Open3D"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/open3d_tf_ops.so"
  IMPORTED_SONAME_RELEASE "open3d_tf_ops.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS Open3D::open3d_tf_ops )
list(APPEND _IMPORT_CHECK_FILES_FOR_Open3D::open3d_tf_ops "${_IMPORT_PREFIX}/lib/open3d_tf_ops.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
