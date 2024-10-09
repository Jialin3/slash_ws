#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "linefit_ground_segmentation::linefit_ground_segmentation" for configuration ""
set_property(TARGET linefit_ground_segmentation::linefit_ground_segmentation APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(linefit_ground_segmentation::linefit_ground_segmentation PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblinefit_ground_segmentation.so"
  IMPORTED_SONAME_NOCONFIG "liblinefit_ground_segmentation.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS linefit_ground_segmentation::linefit_ground_segmentation )
list(APPEND _IMPORT_CHECK_FILES_FOR_linefit_ground_segmentation::linefit_ground_segmentation "${_IMPORT_PREFIX}/lib/liblinefit_ground_segmentation.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
