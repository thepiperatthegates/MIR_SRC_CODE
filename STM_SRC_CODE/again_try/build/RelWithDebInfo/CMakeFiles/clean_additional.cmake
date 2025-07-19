# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "RelWithDebInfo")
  file(REMOVE_RECURSE
  "/Users/mac/Documents/again_try/CM4/build"
  "/Users/mac/Documents/again_try/CM7/build"
  )
endif()
