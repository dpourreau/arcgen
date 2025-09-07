/**
 * @file   test_main.cpp
 * @brief  Minimal GoogleTest entry point (kept for symmetry/flags).
 */
#include <gtest/gtest.h>

int main (int argc, char **argv)
{
    ::testing::InitGoogleTest (&argc, argv);
    return RUN_ALL_TESTS ();
}
