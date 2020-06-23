add_library(parson OBJECT "")

target_sources(
	parson
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/src/parson.c
)

target_include_directories(
	parson
	PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

add_library(base64 OBJECT "")

target_sources(
	base64
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/src/base64.c
)

target_include_directories(
	base64
	PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

add_library(tinymt32 OBJECT "")

target_sources(
	tinymt32
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/src/tinymt32.c
)

target_include_directories(
	tinymt32
	PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)