
add_executable(boot "")
target_sources(
	boot
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/src/boot.c
)

target_link_libraries(
	boot
	PRIVATE
	1302gw
)

set_target_properties(
	boot
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# add the install targets
install (
	TARGETS boot
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
