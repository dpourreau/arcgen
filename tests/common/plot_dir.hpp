#pragma once
/**
 * @file   plot_dir.hpp
 * @brief  Centralized, build-folder-relative plotting paths for tests.
 *
 * All tests should create plots via this helper, so artifacts always land
 * inside the **current build subfolder** (CTest launches from there).
 */

#include <filesystem>
#include <initializer_list>
#include <string>
#include <string_view>
#include <system_error>

namespace test_helpers
{
    /**
     * @brief Root directory for plots: <current_workdir>/plots.
     * @return Absolute path to the plots root (created if missing).
     */
    inline std::filesystem::path plotsRoot ()
    {
        std::filesystem::path root = std::filesystem::current_path () / "plots";
        std::error_code ec;
        std::filesystem::create_directories (root, ec);
        (void)ec;
        return root;
    }

    /**
     * @brief Build a plot file path, creating intermediate directories.
     * @param subdirs   Hierarchy below the plots root, e.g. { "skeleton", "global" }.
     * @param filename  Final filename, e.g. "ok_42.svg".
     * @return Full path to the output file.
     */
    inline std::filesystem::path plotFile (std::initializer_list<std::string_view> subdirs, std::string_view filename)
    {
        auto dir = plotsRoot ();
        for (auto s : subdirs)
            dir /= std::string (s);

        std::error_code ec;
        std::filesystem::create_directories (dir, ec);
        (void)ec;

        return dir / std::string (filename);
    }

} // namespace test_helpers
