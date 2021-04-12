
# GTest
add_library(GTest_in STATIC gtest_recons/gtest.h gtest_recons/gtest-all.cc)
set_target_properties(GTest_in PROPERTIES LINK_FLAGS "-pthread")

# TinyPly
add_library(tinyply STATIC tinyply/tinyply.cpp tinyply/tinyply.h)