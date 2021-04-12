#include <gtest_recons/gtest.h>


int main(int argc, char **argv)
{
    srand(0);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}