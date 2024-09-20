/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_polygon2_h_included
#define sixit_geometry_polygon2_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/line_segment.h"

#include <vector>

namespace sixit
{
namespace geometry
{
    template <typename fp>
    struct polygon2
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::polygon2<fp>>;
        friend struct indexed_polygon2<fp>;
        friend struct bounds2<fp>;
        friend struct shape2<fp>;
        friend struct shape2_base<fp>;
        template<typename T1, typename T2> friend struct low_level::indexed_vector;
        using underlying_vertices_type = std::vector<point2<fp>>;
      private:
        std::vector<point2<fp>> vertices;
        inline void add_vertex(const point2<fp>& p);

      public:
        inline polygon2(const sixit::rw::comparsers::constructor_for_rw_tag&) {};
        inline polygon2(const underlying_vertices_type& p) : vertices{ p } {}
        inline polygon2(const underlying_vertices_type&& p) : vertices{ std::move(p) } {}
        inline polygon2(const triangle2<fp>& tri);
        inline int64_t n_vertices() const;
        inline point2<fp> vertex(int64_t idx) const;
        inline size_t n_edges() const;
        inline line_segment2<fp> edge(size_t idx) const;
        inline size_t n_faces() const;
        inline triangle2<fp> face(size_t idx) const;
        inline ray2<fp> bisector(int64_t idx) const;
        inline int vertex_concavity(int vertex_index) const;
        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square() const;
        inline bounds2<fp> bounds() const;
        inline int normal() const;

        inline void to_triangles(std::vector<triangle2<fp>>&) const;

        inline bool is_inside(const polygon2<fp>& other) const;

        template <typename P2>
        inline bool is_inside(const P2& point) const;

        inline bool is_inside_combinable(const point2<fp>& point) const;
        inline bool strict_intersects(const polygon2& other) const;
        inline std::vector<point2<fp>> strict_intersections(const polygon2& other) const;

        template<typename LS2>
        inline std::vector<point2<fp>> strict_intersections(const LS2& ls) const;

        template <typename P2>
        inline bool is_point_in_edge(const P2& point) const;

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"polygon2">(comparser, obj);
            sixit::rw::read_write<&polygon2::vertices, "vertices", sixit::rw::VofSTRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };
};

namespace geometry 
{
  using polygon2f = polygon2<float>;
}
};

#endif //sixit_geometry_polygon2_h_included

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
