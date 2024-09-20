/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/
    
#ifndef sixit_geometry_indexed_polygon2_h_included
#define sixit_geometry_indexed_polygon2_h_included
    
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/polygon2.h"
#include "sixit/geometry/point.h"

#include "sixit/core/lwa.h"
#include <vector>
#include <map>

class imp_scene_to_mesh3_converter;

namespace R3DSystem_Designer
{
    class imp_scene_to_mesh3_converter;
}

namespace sixit
{
namespace geometry
{
    template <typename fp>
    struct indexed_polygon2 : low_level::indexed_vector<point2<fp>, polygon2<fp>>
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_polygon2<fp>>;
        friend struct polygon2<fp>;
        friend struct indexed_point2<fp>;
        friend imp_scene_to_mesh3_converter;
        friend R3DSystem_Designer::imp_scene_to_mesh3_converter;
    private:
        inline std::vector<indexed_point2<fp>> get_indexed_points() const;
        inline indexed_polygon2(low_level::reference_container<point2<fp>>* ref, const std::vector<size_t>& indexes);

    public:
    
        inline indexed_polygon2(const indexed_triangle2<fp>& tri);
        inline int64_t n_vertices() const;
        inline indexed_point2<fp> vertex(int64_t idx) const;
        inline size_t n_edges() const;
        inline indexed_line_segment2<fp> edge(size_t idx) const;
        inline ray2<fp> bisector(int64_t idx) const;

        inline sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square() const;
        inline bounds2<fp> bounds() const;
        inline int normal() const;

        template <typename T2>
        inline void to_triangles(std::vector<T2>& res) const;

        template <typename P2>
        inline bool is_inside(const P2& point) const;
                
        enum overlap_topology { has_common_vertex = 0x01, has_common_edge = 0x02, overlapping = 0x03 };
    
        inline overlap_topology overlaps(const indexed_triangle2<fp>& tri) const;
        inline overlap_topology overlaps(const indexed_polygon2& p) const;
        inline void join(const indexed_triangle2<fp>& tri);
        inline void join(const indexed_polygon2& other);

    private:
        template <typename T1, typename T2>
        static inline std::map<int64_t, std::vector<int64_t>> common_indices(const T1& a, const T2& b);
        static inline bool segments_intersect(const indexed_line_segment2<fp>& ls1, const indexed_line_segment2<fp>& ls2);
    };
}

namespace geometry 
{
    // aliases 
    using indexed_polygon2f = indexed_polygon2<float>;
}

}

#endif //sixit_geometry_indexed_polygon2_h_included

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