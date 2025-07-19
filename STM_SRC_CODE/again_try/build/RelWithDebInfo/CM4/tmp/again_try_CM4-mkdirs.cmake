# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/mac/Documents/again_try/CM4")
  file(MAKE_DIRECTORY "/Users/mac/Documents/again_try/CM4")
endif()
file(MAKE_DIRECTORY
  "/Users/mac/Documents/again_try/CM4/build"
  "/Users/mac/Documents/again_try/build/RelWithDebInfo/CM4"
  "/Users/mac/Documents/again_try/build/RelWithDebInfo/CM4/tmp"
  "/Users/mac/Documents/again_try/build/RelWithDebInfo/CM4/src/again_try_CM4-stamp"
  "/Users/mac/Documents/again_try/build/RelWithDebInfo/CM4/src"
  "/Users/mac/Documents/again_try/build/RelWithDebInfo/CM4/src/again_try_CM4-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/mac/Documents/again_try/build/RelWithDebInfo/CM4/src/again_try_CM4-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/mac/Documents/again_try/build/RelWithDebInfo/CM4/src/again_try_CM4-stamp${cfgdir}") # cfgdir has leading slash
endif()
