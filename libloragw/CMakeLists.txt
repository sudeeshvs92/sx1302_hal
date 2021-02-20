set(TARGET 1302gw)

add_library(${TARGET} "")

# -- add additional debug options
# Set the DEBUG_* to 1 to activate debug mode in individual modules.
# Warning: that makes the module *very verbose*, do not use for production
option(DEBUG_AUX "Active debug mode in AUX module" OFF)
option(DEBUG_COM "Active debug mode in COM module" OFF)
option(DEBUG_MCU "Active debug mode in MCU module" OFF)
option(DEBUG_I2C "Active debug mode in I2C module" OFF)
option(DEBUG_REG "Active debug mode in REG module" OFF)
option(DEBUG_HAL "Active debug mode in HAL module" OFF)
option(DEBUG_LBT "Active debug mode in LBT module" OFF)
option(DEBUG_GPS "Active debug mode in GPS module" OFF)
option(DEBUG_RAD "Active debug mode in RAD module" OFF)
option(DEBUG_CAL "Active debug mode in CAL module" OFF)
option(DEBUG_SX1302 "Active debug mode in SX1302 module" OFF)
option(DEBUG_FTIME "Active debug mode in FTIME module" OFF)

message("-- Build with debug AUX: ${DEBUG_AUX}")
message("-- Build with debug COM: ${DEBUG_COM}")
message("-- Build with debug MCU: ${DEBUG_MCU}")
message("-- Build with debug I2C: ${DEBUG_I2C}")
message("-- Build with debug REG: ${DEBUG_REG}")
message("-- Build with debug HAL: ${DEBUG_HAL}")
message("-- Build with debug LBT: ${DEBUG_LBT}")
message("-- Build with debug GPS: ${DEBUG_GPS}")
message("-- Build with debug RAD: ${DEBUG_RAD}")
message("-- Build with debug CAL: ${DEBUG_CAL}")
message("-- Build with debug SX1302: ${DEBUG_SX1302}")
message("-- Build with debug FTIME: ${DEBUG_FTIME}")

# --  add the compile options
target_compile_options(
	${TARGET}
	PRIVATE
	-Werror
	-Wall
	-Wextra
)

# -- add source code
target_sources(
	${TARGET}
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_ad5338r.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_aux.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_cal.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_com.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_debug.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_gps.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_hal.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_i2c.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_lbt.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_mcu.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_reg.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_spi.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_stts751.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_sx125x.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_sx1250.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_sx1261.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_sx1302.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_sx1302_rx.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_sx1302_timestamp.c
	${CMAKE_CURRENT_LIST_DIR}/src/loragw_usb.c
	${CMAKE_CURRENT_LIST_DIR}/src/sx125x_com.c
	${CMAKE_CURRENT_LIST_DIR}/src/sx125x_spi.c
	${CMAKE_CURRENT_LIST_DIR}/src/sx1250_com.c
	${CMAKE_CURRENT_LIST_DIR}/src/sx1250_spi.c
	${CMAKE_CURRENT_LIST_DIR}/src/sx1250_usb.c
	${CMAKE_CURRENT_LIST_DIR}/src/sx1261_com.c
	${CMAKE_CURRENT_LIST_DIR}/src/sx1261_spi.c
	${CMAKE_CURRENT_LIST_DIR}/src/sx1261_usb.c
	$<TARGET_OBJECTS:parson>
	$<TARGET_OBJECTS:base64>
	$<TARGET_OBJECTS:tinymt32>
)

# --  add the public headers
set (
	${TARGET}_PUBLIC_HEADERS
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_aux.h
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_gps.h
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_hal.h
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_reg.h
	# TODO: update to v2.0.0
)

target_include_directories(
	${TARGET}
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/
	${CMAKE_CURRENT_LIST_DIR}/inc
	# we use the library parson only, because the three library share the same include directory
	$<TARGET_PROPERTY:parson,INTERFACE_INCLUDE_DIRECTORIES>
	PUBLIC
	$<INSTALL_INTERFACE:include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

configure_file(${CMAKE_CURRENT_LIST_DIR}/${TARGET}_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/config.h" @ONLY)

target_link_libraries(
	${TARGET}
	PUBLIC
	Threads::Threads
	m
)

set_target_properties(${TARGET} PROPERTIES VERSION ${PROJ}_VERSION_SHORT})
set_target_properties(${TARGET} PROPERTIES SOVERSION ${PROJ}_VERSION_SHORT})
set_target_properties(${TARGET} PROPERTIES PUBLIC_HEADER "${CMAKE_CURRENT_BINARY_DIR}/config.h;${${TARGET}_PUBLIC_HEADERS}")

# --  add the install targets
install (TARGETS
	${TARGET}
	EXPORT ${TARGET}_targets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${TARGET}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${TARGET}
)

# -- add pkg config file
configure_file ("${CMAKE_CURRENT_LIST_DIR}/${TARGET}.pc.in" "${PROJECT_BINARY_DIR}/${TARGET}.pc" @ONLY)
install (FILES ${PROJECT_BINARY_DIR}/${TARGET}.pc DESTINATION lib/pkgconfig)

# -- write cmake package config file
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/${TARGET}/${TARGET}-config-version.cmake"
	VERSION ${PROJ}_VERSION_SHORT}
	COMPATIBILITY AnyNewerVersion
)

export(EXPORT ${TARGET}_targets
	FILE "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}/${TARGET}-targets.cmake"
	NAMESPACE Semtech::
	)

configure_file(${PROJECT_SOURCE_DIR}/cmake/${TARGET}-config.cmake
	"${CMAKE_CURRENT_BINARY_DIR}/${TARGET}/${TARGET}-config.cmake"
	COPYONLY
	)

set(ConfigPackageLocation lib/cmake/${TARGET})

install(EXPORT ${TARGET}_targets
	FILE ${TARGET}-targets.cmake
	NAMESPACE Semtech::
	DESTINATION ${ConfigPackageLocation}
	)

install(
	FILES ${PROJECT_SOURCE_DIR}/cmake/${TARGET}-config.cmake "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}/${TARGET}-config-version.cmake"
	DESTINATION ${ConfigPackageLocation}
	COMPONENT Devel
)

# -- add test programs
foreach(
	TEST
	test_loragw_cal_sx125x
	test_loragw_capture_ram
	test_loragw_com
	test_loragw_com_sx1250
	test_loragw_com_sx1261
	test_loragw_counter
	test_loragw_gps
	test_loragw_hal_rx
	test_loragw_hal_tx
	test_loragw_i2c
	test_loragw_reg
	test_loragw_sx1261_rssi
	test_loragw_toa
	)

	add_executable(${TEST} "")

	target_sources(${TEST}
		PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/tst/${TEST}.c
		)

	target_include_directories(${TEST}
		PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
		${CMAKE_CURRENT_LIST_DIR}/inc
		${CMAKE_CURRENT_BINARY_DIR}
		)

	target_link_libraries(${TEST}
		PRIVATE
		1302gw
		)

	install (
		TARGETS ${TEST}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)

endforeach()