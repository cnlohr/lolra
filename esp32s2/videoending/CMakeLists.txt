# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)


add_custom_target(stub_bootload
	# If the swadge is running reboot it into the bootloader.
	COMMAND make -C ../tools/reboot_into_bootloader reboot
	COMMAND sleep 0.5
)
#add_dependencies(flash before_flash)
add_custom_target(stub_run
	#Sometimes the swadge needs a kick to get rebooted
	COMMAND sleep 0.5
	COMMAND esptool.py --before no_reset --after no_reset load_ram ../tools/bootload_reboot_stub/bootload_reboot_stub.bin
)


project(usb_sandbox)


