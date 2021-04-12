file(GLOB_RECURSE mainSourceFiles ${CMAKE_CURRENT_LIST_DIR}/*.cpp ${CMAKE_CURRENT_LIST_DIR}/*.h)
add_library(ransacLib STATIC ${mainSourceFiles})