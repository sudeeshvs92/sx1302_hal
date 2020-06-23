set(APP lora_pkt_fwd)

add_executable(${APP} "")

# --  add the compile options
target_compile_options(
	${APP}
	PRIVATE
	-Wall
	-Wextra
)

target_compile_definitions(
	${APP}
	PRIVATE
	VERSION_STRING="${${PROJ}_VERSION_SHORT}"
)

target_sources(
	${APP}
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/src/jitqueue.c
	${CMAKE_CURRENT_LIST_DIR}/src/lora_pkt_fwd.c
)

target_include_directories(
	${APP}
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/inc
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/inc>
	$<TARGET_PROPERTY:parson,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(
	${APP}
	PUBLIC
	Threads::Threads
	m
	1302gw
)

set_target_properties(${TARGET} PROPERTIES VERSION ${${PROJ}_VERSION_SHORT})
set_target_properties(${TARGET} PROPERTIES SOVERSION ${${PROJ}_VERSION_SHORT})

# add the install targets
install (
	TARGETS ${APP}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)