#pragma once
/**
 * @file
 * @brief Lightweight SVG visualizer for workspaces, skeleton graphs, and sampled paths.
 *
 * Header-only helper to export simple SVG figures that show:
 *  - the valid region (with holes),
 *  - a skeleton graph (edges + optional vertex dots),
 *  - a sampled path,
 *  - start/goal poses, and
 *  - axes.
 *
 * The visualizer auto-fits the world extents into a square canvas and correctly flips
 * the Y axis for SVG (world Y up).
 */

#include <arcgen.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/range/iterator_range.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace arcgen::utils
{
    /**
     * @brief Simple color palette for the SVG output.
     */
    struct Palette
    {
        std::string canvasBackground = "#373639ff"; ///< Background color for the entire generated image.
        std::string bg = "#b55252ff";               ///< Fill color for holes.
        std::string regionFill = "#fafafaff";       ///< Fill color for the valid region.
        std::string regionStroke = "#1f1f20ff";     ///< Stroke color for region outlines.
        std::string skeletonEdge = "#6d737fff";     ///< Stroke color for skeleton edges.
        std::string pathForward = "#88d8b0";        ///< Stroke color for forward-motion segments.
        std::string pathReverse = "#c088d8ff";      ///< Stroke color for reverse-motion segments.
        std::string pathNeutral = "#dcdfd3ff";      ///< Stroke color for neutral / indeterminate segments.
        std::string startPose = "#033aa9ff";        ///< Fill color for the start pose marker.
        std::string goalPose = "#03a940ff";         ///< Fill color for the goal pose marker.
        std::string axes = "#c0c5ce";               ///< Stroke color for axes.
        std::string robotFill = "#375765ff";        ///< Fill color for robot footprint.
        std::string robotGhost = "#375765ff";       ///< Fill color for robot ghost/trail.
        std::string graphEdge = "#ffb300ff";        ///< Stroke color for graph edges / resampled points.
        std::string anchor = "#800080ff";           ///< Fill color for fixed anchors.
    };

    /**
     * @brief SVG visualizer that auto-fits world geometry to a square canvas.
     *
     * Typical usage:
     * @code
     *   Visualizer viz("out.svg");
     *   viz.drawRegion(workspace);
     *   viz.drawSkeleton(graph);
     *   viz.drawPath(states);
     *   viz.drawStartPose(start);
     *   viz.drawGoalPose(goal);
     *   // viz.finish(); // optional; destructor will close the file as well
     * @endcode
     */
    class Visualizer
    {
      public:
        /**
         * @brief Construct a visualizer bound to an output file.
         * @param filename   Target SVG file path.
         * @param canvasPx   Square canvas size in pixels (width = height = canvasPx).
         * @param palette    Optional color palette (defaults provided).
         */
        explicit Visualizer (std::string_view filename, int canvasPx = 800, Palette palette = {})
            : canvasPx_ (canvasPx), palette_ (std::move (palette)), out_ (std::string (filename), std::ios::trunc)
        {
            assert (out_ && "Cannot open SVG output file");
            resetWorldExtents ();
        }

        /// @brief Flush and close the SVG file if still open.
        ~Visualizer () { finish (); }

        Visualizer (const Visualizer &) = delete;
        Visualizer &operator= (const Visualizer &) = delete;

        /**
         * @brief Manually set a fixed scale (px per world unit). Disables auto-fit scaling.
         * @param s  Scale factor (px/world-unit).
         */
        void setScale (double s)
        {
            scale_ = s;
            fixedScale_ = true;
        }

        /**
         * @brief Manually set a fixed world offset prior to scaling. Disables auto-offset.
         * @param dx  World X offset added before scaling.
         * @param dy  World Y offset added before scaling.
         */
        void setOffset (double dx, double dy)
        {
            offsetX_ = dx;
            offsetY_ = dy;
            userOffset_ = true;
        }

        /**
         * @brief Access the active color palette.
         */
        [[nodiscard]] const Palette &getPalette () const { return palette_; }

        /**
         * @brief Draw Cartesian axes through the canvas center.
         * @param stroke  Stroke width in pixels.
         */
        void drawAxes (double stroke = 0.5)
        {
            ensureHeader ();
            const double cx = canvasWidth_ / 2.0;
            const double cy = canvasHeight_ / 2.0;
            lineSvg (0, cy, canvasWidth_, cy, palette_.axes, stroke);
            lineSvg (cx, 0, cx, canvasHeight_, palette_.axes, stroke);
        }

        /**
         * @brief Draw a polyline from a sequence of states while highlighting motion direction (forward vs. reverse).
         * @param points Sampled states along the path.
         * @param color   Optional stroke color (defaults to @ref Palette::pathForward).
         * @param stroke Stroke width in pixels.
         */
        void drawPath (std::span<const arcgen::core::State> points, std::string color = {}, double stroke = 2.0, double opacity = 0.8)
        {
            if (points.empty ())
                return;

            for (const auto &p : points)
                trackWorld (p.x, p.y);

            ensureHeader ();

            const auto &palette = palette_;
            const auto selectColor = [&] (arcgen::core::DrivingDirection dir) -> const std::string &
            {
                if (!color.empty ())
                    return color;

                switch (dir)
                {
                    case arcgen::core::DrivingDirection::Forward:
                        return palette.pathForward;
                    case arcgen::core::DrivingDirection::Reverse:
                        return palette.pathReverse;
                    default:
                        return palette.pathNeutral;
                }
            };

            auto emitSegment = [&] (std::size_t begin, std::size_t end, arcgen::core::DrivingDirection dir)
            {
                if (end <= begin + 1)
                    return;

                const std::string &strokeColor = selectColor (dir);
                out_ << R"(  <polyline fill="none" stroke=")" << strokeColor << R"(" stroke-width=")" << stroke << R"(" stroke-opacity=")" << opacity << R"(" points=")";
                for (std::size_t idx = begin; idx < end; ++idx)
                {
                    const auto [x, y] = toSvg (points[idx].x, points[idx].y);
                    out_ << x << ',' << y << ' ';
                }
                out_ << "\"/>\n";
            };

            std::size_t segBegin = 0;
            arcgen::core::DrivingDirection segDir = points.front ().direction;

            for (std::size_t idx = 1; idx < points.size (); ++idx)
            {
                const arcgen::core::DrivingDirection dir = points[idx].direction;
                if (dir != segDir)
                {
                    emitSegment (segBegin, idx, segDir);
                    segBegin = idx > 0 ? idx - 1 : idx;
                    segDir = dir;
                }
            }

            emitSegment (segBegin, points.size (), segDir);
        }

        /**
         * @brief Draw a triangular pose marker pointing along @p s.heading.
         * @param s      Pose to draw.
         * @param rPx    Marker size (approximate radius in pixels).
         * @param color  Fill color for the marker.
         */
        void drawPose (const arcgen::core::State &s, double rPx, std::string color)
        {
            trackWorld (s.x, s.y);
            ensureHeader ();

            const auto [cx, cy] = toSvg (s.x, s.y);
            const double dx = std::cos (s.heading) * rPx;
            const double dy = -std::sin (s.heading) * rPx; // flip Y for SVG

            const double ax = cx + dx;
            const double ay = cy + dy;
            const double bx = cx - 0.5 * dx + 0.35 * dy;
            const double by = cy - 0.5 * dy - 0.35 * dx;
            const double cx2 = cx - 0.5 * dx - 0.35 * dy;
            const double cy2 = cy - 0.5 * dy + 0.35 * dx;

            out_ << "  <polygon fill=\"" << color << "\" points=\"" << ax << ',' << ay << ' ' << bx << ',' << by << ' ' << cx2 << ',' << cy2 << "\"/>\n";
        }

        /**
         * @brief Draw the start pose with the palette's start color.
         * @param s      Start pose.
         * @param rPx    Marker size in pixels.
         * @param color  Optional override color.
         */
        void drawStartPose (const arcgen::core::State &s, double rPx = 9.0, std::string color = {})
        {
            if (color.empty ())
                color = palette_.startPose;

            drawPose (s, rPx, color);
        }

        /**
         * @brief Draw the goal pose with the palette's goal color.
         * @param s      Goal pose.
         * @param rPx    Marker size in pixels.
         * @param color  Optional override color.
         */
        void drawGoalPose (const arcgen::core::State &s, double rPx = 9.0, std::string color = {})
        {
            if (color.empty ())
                color = palette_.goalPose;

            drawPose (s, rPx, color);
        }

        // Styled ring writer with fill opacity and stroke opacity parameter.
        template <class Ring>
        void writeRingStyled (const Ring &ring, const std::string &fill, const std::string &stroke, double strokeWidth, double fillOpacity, double strokeOpacity)
        {
            out_ << "  <polygon fill=\"" << fill << "\" fill-opacity=\"" << fillOpacity << "\" stroke=\"" << stroke << "\" stroke-width=\"" << strokeWidth << "\" stroke-opacity=\""
                 << strokeOpacity << "\" points=\"";
            for (auto &v : ring)
            {
                const auto [x, y] = toSvg (v.x (), v.y ());
                out_ << x << ',' << y << ' ';
            }
            out_ << "\"/>\n";
        }

        /**
         * @brief Draw a single polygon with custom style.
         * @param poly         CCW, closed polygon.
         * @param fill         Fill color (e.g., "#ff0000" or "none").
         * @param stroke       Stroke color.
         * @param strokeWidth  Stroke width in pixels.
         * @param fillOpacity  Opacity for the fill [0..1].
         * @param strokeOpacity Opacity for the stroke [0..1].
         */
        void drawPolygon (const arcgen::planner::geometry::Polygon &poly, const std::string &fill, const std::string &stroke, double strokeWidth = 0.8, double fillOpacity = 1.0,
                          double strokeOpacity = 1.0)
        {
            for (auto &v : poly.outer ())
                trackWorld (v.x (), v.y ());
            for (auto &hole : poly.inners ())
                for (auto &v : hole)
                    trackWorld (v.x (), v.y ());

            ensureHeader ();
            writeRingStyled (poly.outer (), fill, stroke, strokeWidth, fillOpacity, strokeOpacity);
            for (const auto &hole : poly.inners ())
                writeRingStyled (hole, palette_.regionFill, stroke, strokeWidth, 1.0, strokeOpacity);
        }

        /**
         * @brief Draw a multi-polygon with custom style.
         * @param mp           Multi-polygon (each with optional holes).
         * @param fill         Fill color (e.g., "#ff0000" or "none").
         * @param stroke       Stroke color.
         * @param strokeWidth  Stroke width in pixels.
         * @param fillOpacity  Opacity for the fill [0..1].
         * @param strokeOpacity Opacity for the stroke [0..1].
         */
        void drawMultiPolygon (const arcgen::planner::geometry::MultiPolygon &mp, const std::string &fill, const std::string &stroke, double strokeWidth = 0.8,
                               double fillOpacity = 1.0, double strokeOpacity = 1.0)
        {
            for (const auto &poly : mp)
            {
                for (auto &v : poly.outer ())
                    trackWorld (v.x (), v.y ());
                for (auto &hole : poly.inners ())
                    for (auto &v : hole)
                        trackWorld (v.x (), v.y ());
            }
            ensureHeader ();

            for (const auto &poly : mp)
            {
                writeRingStyled (poly.outer (), fill, stroke, strokeWidth, fillOpacity, strokeOpacity);
                for (const auto &hole : poly.inners ())
                    writeRingStyled (hole, palette_.regionFill, stroke, strokeWidth, 1.0, strokeOpacity);
            }
        }

        /**
         * @brief Draw the valid-region polygons (outer rings + holes).
         * @param workspace   Valid region (multi-polygon).
         * @param strokeWidth Stroke width in pixels for region edges.
         */
        void drawRegion (const arcgen::planner::geometry::Workspace &workspace, double strokeWidth = 0.8)
        {
            // Gather world extents first.
            for (const auto &poly : workspace.region ())
            {
                for (auto &v : poly.outer ())
                    trackWorld (v.x (), v.y ());
                for (auto &hole : poly.inners ())
                    for (auto &v : hole)
                        trackWorld (v.x (), v.y ());
            }
            ensureHeader ();

            // Draw outer rings and holes.
            for (const auto &poly : workspace.region ())
            {
                writeRing (poly.outer (), palette_.regionFill, palette_.regionStroke, strokeWidth);
                for (const auto &hole : poly.inners ())
                    writeRing (hole, palette_.bg, palette_.regionStroke, strokeWidth);
            }
        }

        /**
         * @brief Draw a Boost.Graph-like skeleton: edges and optional vertex dots.
         * @tparam Graph       Undirected graph whose vertex property offers x() / y().
         * @param graph        Skeleton graph.
         * @param strokeWidth  Edge stroke width in pixels.
         * @param vertexRadius Radius of filled vertex dots in pixels (set to 0.0 to skip).
         */
        template <class Graph> void drawSkeleton (const Graph &graph, double strokeWidth = 0.6, double vertexRadius = 2.0)
        {
            // 1) Track extents (edges + vertices).
            for (auto e : boost::make_iterator_range (edges (graph)))
            {
                const auto &p = graph[source (e, graph)];
                const auto &q = graph[target (e, graph)];
                trackWorld (p.x (), p.y ());
                trackWorld (q.x (), q.y ());
            }
            for (auto v : boost::make_iterator_range (vertices (graph)))
                trackWorld (graph[v].x (), graph[v].y ());

            ensureHeader ();

            // 2) Draw edges.
            for (auto e : boost::make_iterator_range (edges (graph)))
            {
                const auto &p = graph[source (e, graph)];
                const auto &q = graph[target (e, graph)];
                lineWorld (p.x (), p.y (), q.x (), q.y (), palette_.skeletonEdge, strokeWidth);
            }

            // 3) Draw filled circles at every vertex (if requested).
            if (vertexRadius > 0.0)
                for (auto v : boost::make_iterator_range (vertices (graph)))
                    circleWorld (graph[v].x (), graph[v].y (), vertexRadius, palette_.skeletonEdge);
        }

        /// @brief Finalize the SVG (idempotent). Called automatically by the destructor.
        void finish ()
        {
            if (out_.is_open ())
            {
                out_ << " </g>\n</svg>\n";
                out_.close ();
            }
        }

        /**
         * @brief Draw a world-space line segment.
         * @param x0,y0  World-space start.
         * @param x1,y1  World-space end.
         * @param color  Stroke color.
         * @param width  Stroke width in pixels.
         */
        void lineWorld (double x0, double y0, double x1, double y1, const std::string &color, double width)
        {
            const auto [sx0, sy0] = toSvg (x0, y0);
            const auto [sx1, sy1] = toSvg (x1, y1);
            lineSvg (sx0, sy0, sx1, sy1, color, width);
        }

      private:
        /*──────────────────── header & transforms ──────────────────────*/
        /// @brief Reset world extents to ±∞ sentinels.
        void resetWorldExtents ()
        {
            xMinWorld_ = std::numeric_limits<double>::infinity ();
            xMaxWorld_ = -xMinWorld_;
            yMinWorld_ = xMinWorld_;
            yMaxWorld_ = -xMinWorld_;
        }

        /// @brief Expand stored world extents to include (x,y). Safe to call on const objects.
        void trackWorld (double x, double y) const
        {
            xMinWorld_ = std::min (xMinWorld_, x);
            xMaxWorld_ = std::max (xMaxWorld_, x);
            yMinWorld_ = std::min (yMinWorld_, y);
            yMaxWorld_ = std::max (yMaxWorld_, y);
        }

        /*──────────────────── low-level SVG helpers ────────────────────*/
        /**
         * @brief Write a polygon ring (outer or hole).
         * @tparam Ring       Boost.Geometry ring-like type with x()/y() vertices.
         * @param ring        The ring to write.
         * @param fill        Fill color.
         * @param stroke      Stroke color.
         * @param strokeWidth Stroke width in pixels.
         */
        template <class Ring> void writeRing (const Ring &ring, const std::string &fill, const std::string &stroke, double strokeWidth)
        {
            out_ << "  <polygon fill=\"" << fill << "\" stroke=\"" << stroke << "\" stroke-width=\"" << strokeWidth << "\" points=\"";
            for (auto &v : ring)
            {
                const auto [x, y] = toSvg (v.x (), v.y ());
                out_ << x << ',' << y << ' ';
            }
            out_ << "\"/>\n";
        }

        // Styled ring writer with fill opacity parameter.
        template <class Ring> void writeRingStyled (const Ring &ring, const std::string &fill, const std::string &stroke, double strokeWidth, double fillOpacity)
        {
            out_ << "  <polygon fill=\"" << fill << "\" fill-opacity=\"" << fillOpacity << "\" stroke=\"" << stroke << "\" stroke-width=\"" << strokeWidth << "\" points=\"";
            for (auto &v : ring)
            {
                const auto [x, y] = toSvg (v.x (), v.y ());
                out_ << x << ',' << y << ' ';
            }
            out_ << "\"/>\n";
        }

        /**
         * @brief Draw a canvas-space line segment.
         * @param x0,y0  Canvas-space start.
         * @param x1,y1  Canvas-space end.
         * @param color  Stroke color.
         * @param width  Stroke width in pixels.
         */
        void lineSvg (double x0, double y0, double x1, double y1, const std::string &color, double width)
        {
            out_ << "  <line x1=\"" << x0 << "\" y1=\"" << y0 << "\" x2=\"" << x1 << "\" y2=\"" << y1 << "\" stroke=\"" << color << "\" stroke-width=\"" << width << "\"/>\n";
        }

        /**
         * @brief Draw a filled circle in world space.
         * @param x,y   World-space center.
         * @param rPx   Radius in pixels.
         * @param color Fill color.
         */
        void circleWorld (double x, double y, double rPx, const std::string &color)
        {
            const auto [cx, cy] = toSvg (x, y);
            circleSvg (cx, cy, rPx, color);
        }

        /**
         * @brief Draw a filled circle in canvas coordinates.
         * @param cx,cy Canvas-space center.
         * @param rPx   Radius in pixels.
         * @param color Fill color.
         */
        void circleSvg (double cx, double cy, double rPx, const std::string &color)
        {
            out_ << "  <circle cx=\"" << cx << "\" cy=\"" << cy << "\" r=\"" << rPx << "\" fill=\"" << color << "\" stroke=\"none\"/>\n";
        }

        /*──────────────────── header & transforms ──────────────────────*/
        /// @brief Ensure the SVG header/group has been written (runs auto-fit first if needed).
        void ensureHeader ()
        {
            if (svgOpen_)
                return;
            autoFit ();
            beginSvg ();
        }

        /// @brief Compute an automatic scale/offset to center-fit world geometry into the canvas.
        void autoFit ()
        {
            if (fixedScale_ || !std::isfinite (xMinWorld_))
                return;

            const double dx = xMaxWorld_ - xMinWorld_;
            const double dy = yMaxWorld_ - yMinWorld_;
            const double half = std::max (dx, dy) / 2.0;
            if (half < 1e-9)
                return; // degenerate

            scale_ = (1.0 - margin_) * canvasPx_ / (2.0 * half);

            if (!userOffset_)
            {
                offsetX_ = -(xMinWorld_ + xMaxWorld_) / 2.0;
                offsetY_ = -(yMinWorld_ + yMaxWorld_) / 2.0;
            }
        }

        /// @brief Begin the SVG document and open a root <g> group.
        void beginSvg ()
        {
            canvasWidth_ = canvasHeight_ = canvasPx_;
            out_ << R"(<?xml version="1.0"?>)"
                 << "\n<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << canvasWidth_ << "\" height=\"" << canvasHeight_ << "\" viewBox=\"0 0 " << canvasWidth_ << ' '
                 << canvasHeight_ << "\">\n"
                 << "  <rect width=\"" << canvasWidth_ << "\" height=\"" << canvasHeight_ << "\" fill=\"" << palette_.canvasBackground << "\"/>\n"
                 << "  <g>\n";
            svgOpen_ = true;
        }

        /**
         * @brief Convert world coordinates to canvas (SVG) coordinates.
         * @param xw  World X.
         * @param yw  World Y.
         * @return Pair {xCanvas, yCanvas}. Y is flipped for SVG.
         */
        [[nodiscard]] std::pair<double, double> toSvg (double xw, double yw) const
        {
            const double x = (xw + offsetX_) * scale_ + canvasWidth_ / 2.0;
            const double y = (yw + offsetY_) * scale_ + canvasHeight_ / 2.0;
            return {x, canvasHeight_ - y}; // flip Y
        }

        /*────────────────────────── data ───────────────────────────────*/
        int canvasPx_;      ///< Canvas size (square) in pixels.
        Palette palette_;   ///< Active color palette.
        std::ofstream out_; ///< Output SVG stream.

        mutable double xMinWorld_; ///< Tracked world extents (min X).
        mutable double xMaxWorld_; ///< Tracked world extents (max X).
        mutable double yMinWorld_; ///< Tracked world extents (min Y).
        mutable double yMaxWorld_; ///< Tracked world extents (max Y).

        double scale_ = 1.0;   ///< Pixels per world unit.
        double offsetX_ = 0.0; ///< World X offset applied before scaling.
        double offsetY_ = 0.0; ///< World Y offset applied before scaling.
        double margin_ = 0.05; ///< Fractional margin for auto-fit.

        bool fixedScale_ = false; ///< If true, auto-fit scaling is disabled.
        bool userOffset_ = false; ///< If true, auto-fit offset is disabled.

        bool svgOpen_ = false; ///< Whether the SVG header/group is open.
        int canvasWidth_ = 0;  ///< Canvas width in pixels.
        int canvasHeight_ = 0; ///< Canvas height in pixels.
    };

} // namespace arcgen::utils
