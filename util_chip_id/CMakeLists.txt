
add_executable(chip_id "")
target_sources(
	chip_id
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/src/chip_id.c
)

target_link_libraries(
	chip_id
	PRIVATE
	1302gw
)

set_target_properties(
	chip_id
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# add the install targets
install (
	TARGETS chip_id
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
