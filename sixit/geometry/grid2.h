/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_grid2_h_included
#define sixit_geometry_grid2_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/core/guidelines.h"
#include "sixit/geometry/line_segment_impl.h"
#include "sixit/geometry/sixit_mathf.h"

#include <unordered_map>

namespace sixit
{
namespace geometry
{
    struct grid2_cell
    {
        grid2_cell() = default;
        grid2_cell(int64_t x, int64_t y) : x(x) , y(y) {}

        bool operator<(const grid2_cell& other) const { return (x != other.x) ? (x < other.x) : (y < other.y); }
        bool operator==(const grid2_cell& other) const { return (x == other.x) && (y == other.y); }

        int64_t x = 0;
        int64_t y = 0;
    };

    template <typename fp>
    class grid2
    {
    public:
        grid2(fp step_x, fp step_y) : step_x_(step_x > fp(0.f) ? step_x : fp(1.0f)), step_y_(step_y > fp(0.f) ? step_y : fp(1.0f))
        { }

        inline fp get_step_x() const { return step_x_; }
        inline fp get_step_y() const { return step_y_; }
        inline grid2_cell point2_to_grid_cell(const point2<fp>& real) const;

        inline std::vector<grid2_cell> line_segment2_to_grid_cells(const line_segment2<fp>& ls) const;
        inline std::vector<grid2_cell> triangle2_to_grid_cells(const triangle2<fp>& tri) const;
        inline std::vector<grid2_cell> triangle2_bounds_to_grid_cells(const triangle2<fp>& tri) const;

        // -- Iterators
        class base_iterator
        {
        public:
            using reference = grid2_cell&;
            using pointer = grid2_cell*;

            inline base_iterator() : current_cell_(INT64_MAX, INT64_MAX) { }

            inline reference operator*() { return current_cell_; }
            inline pointer operator->() { return &current_cell_; }

            inline bool operator==(const base_iterator& other) const { return current_cell_ == other.current_cell_; }
            inline bool operator!=(const base_iterator& other) const { return !(*this == other); }

        protected:
            inline void set_current_cell(const grid2_cell& cell) { current_cell_ = cell; }
            inline void move_to_end() { current_cell_ = grid2_cell(INT64_MAX, INT64_MAX); }

        private:
            grid2_cell current_cell_;
        };
        
        class iterator : public base_iterator
        {
        public:
            inline iterator(const grid2<fp>& grid, const point2<fp>& coord);
            inline iterator& operator++();
        };

        class triangle_iterator : public base_iterator
        {
        public:
            inline triangle_iterator(const grid2<fp>& grid, const triangle2<fp>& tri, bool with_bounds = false);
            inline triangle_iterator& operator++();

        private:
            std::vector<grid2_cell> cells_;
            size_t current_cell_index_ = 0;
        };

        class line_segment_iterator : public base_iterator
        {
        public:
            inline line_segment_iterator(const grid2<fp>& grid, const line_segment2<fp>& line);
            inline line_segment_iterator(const grid2<fp>& grid, const line_segment2<fp>& line, const bounds2<fp>& box);
            inline line_segment_iterator& operator++();

        private:
            inline void initialize_iterator(const grid2<fp>& grid, const line_segment2<fp>& ls);
            inline line_segment2<fp> clip_line_segment(const line_segment2<fp>& ls, const bounds2<fp>& box);

        private:
            std::vector<grid2_cell> cells_;
            size_t current_cell_index_ = 0;
        };

        inline iterator cells_begin(const point2<fp>& coord) const { return iterator(*this, coord); }
        inline triangle_iterator cells_begin(const triangle2<fp>& tri) const { return triangle_iterator(*this, tri); }
        inline line_segment_iterator cells_begin(const line_segment2<fp>& coord) { return line_segment_iterator(*this, coord); }
        inline line_segment_iterator cells_begin(const line_segment2<fp>& coord, const bounds2<fp>& box) { return line_segment_iterator(*this, coord, box); }

        inline base_iterator cells_end() const { return base_iterator(); }

    protected:
        // Convert real-world coordinates to the nearest floor grid coordinates
        inline int64_t to_floor_grid_x(fp real_x) const { return sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::floor(real_x / step_x_)); }
        inline int64_t to_floor_grid_y(fp real_y) const { return sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::floor(real_y / step_y_)); }
        inline point2<fp> to_floor_grid(const point2<fp>& point) const { return point2<fp>(fp(sixit::guidelines::precision_cast<float>(to_floor_grid_x(point.vec().x))), fp(sixit::guidelines::precision_cast<float>(to_floor_grid_y(point.vec().y)))); }

        // Convert real-world coordinates to the grid coordinates
        inline fp to_grid_x(fp real_x) const { return real_x / step_x_; }
        inline fp to_grid_y(fp real_y) const { return real_y / step_y_; }
        inline point2<fp> to_grid(const point2<fp>& point) const { return point2<fp>(
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(to_grid_x(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(point.vec().x))), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(to_grid_y(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(point.vec().y)))); }

    private:
        fp step_x_ = 0.f; // Step size along X axis
        fp step_y_ = 0.f; // Step size along Y axis
    };

    // -- grid2 functions implementations
    template <typename fp>
    inline grid2_cell grid2<fp>::point2_to_grid_cell(const point2<fp>& real) const
    {
        return { 
            to_floor_grid_x(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(real.vec().x)), 
            to_floor_grid_y(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(real.vec().y)) };
    }

    template <typename fp>
    inline std::vector<grid2_cell> grid2<fp>::line_segment2_to_grid_cells(const line_segment2<fp>& ls) const
    {
        std::vector<grid2_cell> cells_vector;
        auto set_pixel = [&](int x, int y) { cells_vector.push_back({ x, y }); };

        // Adjust the line segment's endpoints to grid coordinates
        point2<fp> grid_point1 = to_grid(ls.p1().vec());
        point2<fp> grid_point2 = to_grid(ls.p2().vec());

        line_segment2<fp> grid_line(grid_point1, grid_point2);
        grid_line.supercover_line(set_pixel);                                // qualified name is workaround for XCode 15
        std::stable_sort(cells_vector.begin(), cells_vector.end());
        cells_vector.erase(std::unique(cells_vector.begin(), cells_vector.end()), cells_vector.end());
        return cells_vector;
    }

    template <typename fp>
    inline std::vector<grid2_cell> grid2<fp>::triangle2_to_grid_cells(const triangle2<fp>& tri) const
    {
        std::unordered_map<int, std::pair<int, int>> scan_lines; // y, x's
        int cells_size = 0;
        auto set_pixel = [&](int x, int y) { 
                if (scan_lines.find(y) == scan_lines.end()) 
                {
                    scan_lines[y] = std::make_pair(x, x);
                    cells_size += 1;
                }
                else 
                {
                    if (scan_lines[y].first > x)
                    {
                        cells_size += scan_lines[y].first - x;
                        scan_lines[y].first = x;
                    }
                    if (scan_lines[y].second < x)
                    {
                        cells_size += x - scan_lines[y].second;
                        scan_lines[y].second = x;
                    }
                }
            };

        // Adjust the triangle's vertices to grid coordinates
        point2<fp> grid_vertex1 = to_grid(tri.vertex(0));
        point2<fp> grid_vertex2 = to_grid(tri.vertex(1));
        point2<fp> grid_vertex3 = to_grid(tri.vertex(2));
        line_segment2<fp>({ grid_vertex1 , grid_vertex2 }).supercover_line(set_pixel);           // qualified name is workaround for XCode 15
        line_segment2<fp>({ grid_vertex2 , grid_vertex3 }).supercover_line(set_pixel);
        line_segment2<fp>({ grid_vertex3 , grid_vertex1 }).supercover_line(set_pixel);

        std::vector<grid2_cell> cells;
        cells.reserve(cells_size);
        for (const auto& [y, x_set] : scan_lines)
        {
            for (auto current_x = x_set.first; current_x <= x_set.second; current_x++)
            {
                cells.emplace_back(current_x, y);
            }
        }

        return cells;
    }

    template <typename fp>
    inline std::vector<grid2_cell> grid2<fp>::triangle2_bounds_to_grid_cells(const triangle2<fp>& tri) const
    {
        const auto bounds = tri.bounds();
        // TODO: NEED TO REFACTOR!!!!
        int64_t minx = low_level::mathf::fp2int(low_level::mathf::floor(to_grid_x(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(bounds.minn.vec().x))));
        int64_t miny = low_level::mathf::fp2int(low_level::mathf::floor(to_grid_y(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(bounds.minn.vec().y))));
        int64_t maxx = low_level::mathf::fp2int(low_level::mathf::ceil(to_grid_x(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter> (bounds.maxx.vec().x))));
        int64_t maxy = low_level::mathf::fp2int(low_level::mathf::ceil(to_grid_y(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter> (bounds.maxx.vec().y))));

        std::vector< grid2_cell> cells;
        cells.reserve((maxx - minx + 1) * (maxy - miny + 1));
        for (int64_t x = minx; x <= maxx; ++x)
            for (int64_t y = miny; y <= maxy; ++y)
                cells.push_back({ x,y });
        return cells;
    }
    // -- iterator functions implementations
    template <typename fp>
    inline grid2<fp>::iterator::iterator(const grid2<fp>& grid, const point2<fp>& coord) : base_iterator()
    {
        this->set_current_cell(grid.point2_to_grid_cell(coord));
    }

    template <typename fp>
    inline typename grid2<fp>::iterator& grid2<fp>::iterator::operator++()
    {
        this->move_to_end(); // because point2 iterator will return only one cell
        return *this;
    }

    // -- triangle_iterator functions implementations
    template <typename fp>
    inline grid2<fp>::triangle_iterator::triangle_iterator(const grid2<fp>& grid, const triangle2<fp>& tri, bool with_bounds) : base_iterator()
    {
        cells_ = with_bounds ? grid.triangle2_bounds_to_grid_cells(tri) : grid.triangle2_to_grid_cells(tri);

        // Sorting cells primarily by x and secondarily by y
        std::sort(cells_.begin(), cells_.end());

        if (!cells_.empty())
            this->set_current_cell(cells_[0]);
        else
            this->move_to_end();
    }

    template <typename fp>
    inline typename grid2<fp>::triangle_iterator& grid2<fp>::triangle_iterator::operator++()
    {
        if (current_cell_index_ < cells_.size() - 1)
        {
            ++current_cell_index_;
            this->set_current_cell(cells_[current_cell_index_]);
        }
        else
        {
            // Reached the end, so set to sentinel value.
            this->move_to_end();
        }
        return *this;
    }

    // -- line_segment_iterator functions implementations
    template <typename fp>
    inline grid2<fp>::line_segment_iterator::line_segment_iterator(const grid2<fp>& grid, const line_segment2<fp>& ls)
        : base_iterator()
    {
        initialize_iterator(grid, ls);
    }

    template <typename fp>
    inline grid2<fp>::line_segment_iterator::line_segment_iterator(const grid2<fp>& grid, const line_segment2<fp>& ls,
                                                               const bounds2<fp>& box)
        : base_iterator()
    {
        if (!ls.bounds().intersects(box))
        {
            // No intersection. The line segment is completely outside of the box.
            this->move_to_end();
            return;
        }

        // Get the grid cells for the (potentially clipped) line_segment2
        line_segment2 clipped_ls = clip_line_segment(ls, box);

        initialize_iterator(grid, clipped_ls);
    }

    template <typename fp>
    inline typename grid2<fp>::line_segment_iterator& grid2<fp>::line_segment_iterator::operator++()
    {
        if (current_cell_index_ < cells_.size() - 1)
        {
            ++current_cell_index_;
            this->set_current_cell(cells_[current_cell_index_]);
        }
        else
        {
            // Reached the end, so set to sentinel value.
            this->move_to_end();
        }
        return *this;
    }

    template <typename fp>
    void grid2<fp>::line_segment_iterator::initialize_iterator(const grid2<fp>& grid, const line_segment2<fp>& ls)
    {
        cells_ = grid.line_segment2_to_grid_cells(ls);

        // Sorting cells primarily by x and secondarily by y
        std::sort(cells_.begin(), cells_.end());

        if (!cells_.empty())
            this->set_current_cell(cells_[0]);
        else
            this->move_to_end();
    }

    template <typename fp>
    inline line_segment2<fp> grid2<fp>::line_segment_iterator::clip_line_segment(const line_segment2<fp>& ls, const bounds2<fp>& box)
    {
        // Create a clipped version of the line segment
        point2<fp> new_p1 = ls.p1();
        point2<fp> new_p2 = ls.p2();

        // If either point of the line segment is outside the box, clip the segment
        bool p1_inside = box.is_inside(new_p1);
        bool p2_inside = box.is_inside(new_p2);

        if (!p1_inside || !p2_inside)
        {
            // Define the four sides of the bounding box as line segments
            std::array<line_segment2<fp>, 4> box_sides = {
                line_segment2<fp>(box.minn, point2<fp>(box.minn.vec().x, box.maxx.vec().y)),
                line_segment2<fp>(box.maxx, point2<fp>(box.maxx.vec().x, box.minn.vec().y)),
                line_segment2<fp>(box.minn, point2<fp>(box.maxx.vec().x, box.minn.vec().y)),
                line_segment2<fp>(box.maxx, point2<fp>(box.minn.vec().x, box.maxx.vec().y)) };

            std::optional<point2<fp>> first_intersection;
            std::optional<point2<fp>> second_intersection;

            for (const auto& side : box_sides)
            {
                auto intersection_point = ls.intersection(side);
                if (intersection_point.has_value())
                {
                    if (!first_intersection)
                    {
                        first_intersection = intersection_point;
                    }
                    else if (!second_intersection)
                    {
                        second_intersection = intersection_point;
                    }
                }
            }

            if (p1_inside)
            {
                new_p2 = first_intersection.value();
            }
            else if (p2_inside)
            {
                new_p1 = first_intersection.value();
            }
            else
            {
                new_p1 = first_intersection.value();
                new_p2 = second_intersection.value();
            }
        }

        // Get potentially clipped line_segment2
        return line_segment2<fp>(new_p1, new_p2);
    }

} // namespace geometry

namespace geometry 
{
    // aliases 
    using grid2f = grid2<float>;
}


} // namespace sixit

#endif //sixit_geometry_grid2_h_included

/*
The 3-Clause BSD License

Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.

Contributors: Sherry Ignatchenko, Mykhailo Borovyk

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/