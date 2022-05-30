macro(add_etcs_executable APPNAME)
  add_executable(${PROJECT_NAME}_${APPNAME} ${CMAKE_CURRENT_SOURCE_DIR}/${APPNAME}_app.cpp)
  target_link_libraries(${PROJECT_NAME}_${APPNAME} PUBLIC ${PROJECT_NAME}_${APPNAME}_lib)
	add_custom_command(TARGET ${PROJECT_NAME}_${APPNAME}
	                   POST_BUILD
	                   COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_DIR:${PROJECT_NAME}_${APPNAME}>/${PROJECT_NAME}_${APPNAME} ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_${APPNAME}
	                   COMMENT "Creating symlink for ${PROJECT_NAME}_${APPNAME}"
	                   VERBATIM)



endmacro()

if (TARGET ${PROJECT_NAME}_sat_lib)
  add_etcs_executable(sat)
endif()


add_etcs_executable(astar)
# add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/heuristic.cpp)
# 	target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_lib)
# 	add_custom_command(TARGET ${PROJECT_NAME}
# 	                   POST_BUILD
# 	                   COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_DIR:${PROJECT_NAME}>/${PROJECT_NAME} ${CMAKE_BINARY_DIR}/${PROJECT_NAME}
# 	                   COMMENT "Creating symlink for ${PROJECT_NAME}"
# 	                   VERBATIM)
