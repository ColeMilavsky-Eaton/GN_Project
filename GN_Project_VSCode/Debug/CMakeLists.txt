cmake_minimum_required(VERSION 3.10)

# Set the project name
project(GN_Project)

# Specify the toolchain file if you're using one
# set(CMAKE_TOOLCHAIN_FILE path/to/your/toolchainfile.cmake)

# Add the executable
add_executable(GN_Project
    main.c
    syscalls.c
    sysmem.c
    stm32g0xx_it.c
    system_stm32g0xx.c
    # Add other source files here
)

# Linker script
set(LINKER_SCRIPT path/to/your/linker_script.ld)
target_link_options(GN_Project PRIVATE "-T${LINKER_SCRIPT}")

# Include directories
target_include_directories(GN_Project PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/path/to/headers
)