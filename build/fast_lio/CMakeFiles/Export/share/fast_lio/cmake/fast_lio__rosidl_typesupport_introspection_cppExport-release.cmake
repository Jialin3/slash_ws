#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fast_lio::fast_lio__rosidl_typesupport_introspection_cpp" for configuration "Release"
set_property(TARGET fast_lio::fast_lio__rosidl_typesupport_introspection_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(fast_lio::fast_lio__rosidl_typesupport_introspection_cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libfast_lio__rosidl_typesupport_introspection_cpp.so"
  IMPORTED_SONAME_RELEASE "libfast_lio__rosidl_typesupport_introspection_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS fast_lio::fast_lio__rosidl_typesupport_introspection_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_fast_lio::fast_lio__rosidl_typesupport_introspection_cpp "${_IMPORT_PREFIX}/lib/libfast_lio__rosidl_typesupport_introspection_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
