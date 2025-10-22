# Declare a header-only interface library
add_library(arch_lib INTERFACE)

# Expose this folder as an include path
target_include_directories(arch_lib INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}   # this is the arch/ folder
)
