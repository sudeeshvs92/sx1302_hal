
add_executable(net_downlink "")
target_sources(
	net_downlink
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/src/net_downlink.c
)

target_include_directories(
	net_downlink
	PRIVATE
	$<TARGET_PROPERTY:parson,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(
	net_downlink
	PRIVATE
	1302gw
)

set_target_properties(
	net_downlink
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# add the install targets
install (
	TARGETS net_downlink
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
