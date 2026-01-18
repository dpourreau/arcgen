#pragma once
/**
 * @file   skeleton.hpp
 * @brief  Lightweight CRTP façade for skeleton generators.
 *
 * A concrete generator must implement:
 *   GraphT generateImpl(const WorkspaceType& workspace) const;
 * which this base forwards to via a const-qualified:
 *   generate(const WorkspaceType& workspace) const.
 */

namespace arcgen::planner::geometry
{
    /**
     * @brief Detection concept for the CRTP hook of @ref SkeletonBase.
     *
     * @tparam D             Candidate derived type.
     * @tparam WorkspaceType Workspace type consumed by the generator.
     */
    template <class D, class WorkspaceType>
    concept HasGenerateImpl = requires (const D &d, const WorkspaceType &w) {
        { d.generateImpl (w) };
    };

    /**
     * @brief CRTP base that forwards to a derived skeleton generator.
     *
     * @tparam Derived       Concrete generator type (must define @c generateImpl()).
     * @tparam WorkspaceType Workspace type consumed by the generator.
     */
    template <typename Derived, typename WorkspaceType> class SkeletonBase
    {
      public:
        /**
         * @brief Generate a skeleton for the given workspace by delegating to the derived class.
         * @param workspace Input valid workspace.
         * @return Graph-like object produced by @c Derived::generateImpl(workspace).
         */
        [[nodiscard]] decltype (auto) generate (const WorkspaceType &workspace) const noexcept (noexcept (static_cast<const Derived &> (*this).generateImpl (workspace)))
            requires HasGenerateImpl<Derived, WorkspaceType>
        {
            return static_cast<const Derived &> (*this).generateImpl (workspace);
        }

      protected:
        /// @brief Defaulted constructor.
        SkeletonBase () = default;
        SkeletonBase (const SkeletonBase &) = default;
        SkeletonBase (SkeletonBase &&) noexcept = default;
        SkeletonBase &operator= (const SkeletonBase &) = default;
        SkeletonBase &operator= (SkeletonBase &&) noexcept = default;
        /// @brief Defaulted destructor (protected to prevent slicing).
        ~SkeletonBase () = default;
    };

} // namespace arcgen::planner::geometry
