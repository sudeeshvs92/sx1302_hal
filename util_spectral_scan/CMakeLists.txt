
add_executable(spectral_scan "")
target_sources(
	spectral_scan
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/src/spectral_scan.c
)

target_link_libraries(
	spectral_scan
	PRIVATE
	1302gw
)

set_target_properties(
	spectral_scan
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# add the install targets
install (
	TARGETS spectral_scan
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
