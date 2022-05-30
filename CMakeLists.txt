cmake_minimum_required(VERSION 3.13)
project(da_etcs
        LANGUAGES CXX
                  DESCRIPTION "DA_ETCS - A Tool for the Generation, Verification and Optimization of ETCS Train Layouts/Schedules"
                  )


set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#configuration options
option(DEPLOY "Configure for deployment")
option(BUILD_TESTS "Also build tests for DA_ETCS project")

#build type settings
set(default_build_type "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
	set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

# Add path for custom modules
list (APPEND CMAKE_MODULE_PATH
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(src)

find_package(Z3)
if(Z3_FOUND AND NOT TARGET z3::z3lib)
	message(STATUS "Found Z3 with version ${Z3_VERSION_STRING}")
	add_library(z3::z3lib IMPORTED INTERFACE)
	set_property(TARGET z3::z3lib PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${Z3_CXX_INCLUDE_DIRS})
	set_property(TARGET z3::z3lib PROPERTY INTERFACE_LINK_LIBRARIES ${Z3_LIBRARIES})
else()
  message(WARNING "Did not find Z3. SAT library and other depending target will not be available")
      endif()
      
# add test code
if (BUILD_TESTS)
	enable_testing()
	include(GoogleTest)
	add_subdirectory(test)
endif ()

# add apps if this is the main project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
	add_subdirectory(apps)
endif()
