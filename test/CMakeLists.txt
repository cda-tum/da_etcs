macro(package_add_test TESTNAME)
	# create an exectuable in which the tests will be stored
	add_executable(${TESTNAME} ${ARGN})
	# link the Google test infrastructure and a default main fuction to the test executable.
        
	target_link_libraries(${TESTNAME} PRIVATE ${PROJECT_NAME}_lib GTest::Main)
	# discover tests
	gtest_discover_tests(${TESTNAME} WORKING_DIRECTORY ${PROJECT_DIR} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_DIR}")
	set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)

	add_custom_command(TARGET ${TESTNAME}
	                   POST_BUILD
	                   COMMENT "Copying files and creating symlinks for ${TESTNAME}"
	                   VERBATIM)
endmacro()

package_add_test(${PROJECT_NAME}_test
                 ${CMAKE_CURRENT_SOURCE_DIR}/test_heuristic.cpp)


