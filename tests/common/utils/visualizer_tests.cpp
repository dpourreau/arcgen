/**
 * @file visualizer_tests.cpp
 * @brief Unit tests for the Visualizer utility class.
 */

#include <filesystem>
#include <gtest/gtest.h>
#include <utils/visualizer.hpp>

using arcgen::core::State;
using arcgen::utils::Palette;
using arcgen::utils::Visualizer;

TEST (VisualizerTest, InstantiationAndFileCreation)
{
    std::filesystem::path filename = "test_viz.svg";
    if (std::filesystem::exists (filename))
    {
        std::filesystem::remove (filename);
    }

    {
        Visualizer viz (filename.string ());
        viz.drawAxes ();
        viz.drawStartPose ({0, 0, 0});
        viz.drawGoalPose ({10, 10, 0});
    }

    EXPECT_TRUE (std::filesystem::exists (filename));
    std::filesystem::remove (filename);
}

TEST (VisualizerTest, CustomPaletteAndScaling)
{
    std::filesystem::path filename = "test_viz_custom.svg";
    Palette p;
    p.bg = "#000000";
    {
        Visualizer viz (filename.string (), 500, p);
        viz.setScale (10.0);
        viz.setOffset (5.0, 5.0);
        std::vector<arcgen::core::State> pth = {{0, 0, 0}, {1, 1, 1}};
        viz.drawPath (pth);
    }
    EXPECT_TRUE (std::filesystem::exists (filename));
    std::filesystem::remove (filename);
}
