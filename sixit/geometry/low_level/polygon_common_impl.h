/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_polygon_common_impl_h_included
#define sixit_geometry_low_level_polygon_common_impl_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/polygon2.h"
#include "sixit/geometry/indexed.h"
#include "sixit/geometry/low_level/line_common_impl.h"

#include <vector>
#include <array>

namespace sixit
{
namespace geometry
{
namespace low_level
{
    template <typename fp>
    struct polygon2_impl
    {
        using class_value = typename point2<fp>::class_value;

        // todo: rewrite to return vector of indexes
        inline static std::vector<triangle2<fp>> triangulate(const std::vector<point2<fp>>& polygon)
        {
            std::vector<point2<fp>> vertices = polygon;
            std::vector<triangle2<fp>> tris;
            tris.reserve(ssize(vertices) - 2);
            while (ssize(vertices) > 3)
            {
                bool ear_found = false;
                for (int i = 0; i < ssize(vertices); ++i)
                {
                    int64_t prev = (i == 0) ? ssize(vertices) - 1 : i - 1;
                    int64_t next = (i == ssize(vertices) - 1) ? 0 : i + 1;
                    const point2<fp>& prv_p = vertices[prev];
                    const point2<fp>& cur_p = vertices[i];
                    const point2<fp>& nxt_p = vertices[next];
                    if (polygon2_impl::is_ear(prv_p, cur_p, nxt_p, vertices))
                    {
                        tris.push_back({prv_p, cur_p, nxt_p});
                        vertices.erase(vertices.begin() + i);
                        ear_found = true;
                        break;
                    }
                }
                assert(ear_found);
            }
            tris.push_back({vertices[0], vertices[1], vertices[2]});
            return tris;
        }

        inline static bool is_ear(const point2<fp>& prv, const point2<fp>& cur, const point2<fp>& nxt, const std::vector<point2<fp>>& vertices)
        {
            for (const auto& p: vertices)
            {
                if (p.vec() != prv.vec() && p.vec() != cur.vec() && p.vec() != nxt.vec())
                {
                    if (tri2_impl<fp>::point_inside(prv, cur, nxt, p))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        inline static bounds2<fp> bounds(const std::vector<point2<fp>>& poly)
        {
            low_level::dimensional_vector2 min = {
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::max_float), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::max_float)};

            low_level::dimensional_vector2 max = {
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::min_float), 
                sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::min_float)};
                
            for(const auto& p: poly)
            {
                min = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::min(p.vec(), min);
                max = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::max(p.vec(), max);
            }
            return bounds2<fp>(min, max);
        }


        inline static sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim> square(const std::vector<point2<fp>>& vertices)
        {
            std::vector<triangle2<fp>> tris = polygon2_impl::triangulate(vertices);
            auto res = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.0f);
            for(const auto& t: tris)
            {
                res += t.square();
            }
            return res;
        }

        inline static int normal(const std::vector<point2<fp>>& vertices)//-1, 0, 1
        {
            auto res = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.0f);

            auto process_cross = [&](const point2<fp>& a, const point2<fp>& b) -> sixit::units::dimensional_scalar<fp, sixit::units::square_meter::dim>
            {
                return a.vec().x * b.vec().y - a.vec().y * b.vec().x;
            };

            for (int i = 0; i < vertices.size() - 1; ++i)
            {
                res += process_cross(vertices[i], vertices[i + 1]);
            }

            res += process_cross(vertices[vertices.size() - 1], vertices[0]);

            if (res < sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.0f))
                return -1;
            if (res > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(0.0f))
                return 1;
            return 0;
        }

        inline static int normal(const std::vector<::sixit::geometry::indexed_point2<fp>>& /*vertices*/)//-1, 0, 1
        {
            // float res = 0.0f;

       //     vertices[0].vec()

            // produces error
            // for (int i = 0; i < vertices.size() - 1; ++i)
            // {
                /*low_level::vector3 cross = vector3::cross(
                    { vertices[i].vec().x, vertices[i].vec().y, 0.0f },
                    { vertices[i + 1].vec().x, vertices[i + 1].vec().y, 0.0f }
                );

                res += cross.z;*/
            // }

          /*  low_level::vector3 cross = vector3::cross(
                { vertices[vertices.size() - 1].vec().x, vertices[vertices.size() - 1].vec().y, 0.0f },
                { vertices[0].vec().x, vertices[0].vec().y, 0.0f });

            res += cross.z;

            if (res < 0.0f)
                return -1;
            if (res > 0.0f)
                return 1;*/
            return 0;
        }
    };
}; // namespace low_level


namespace low_level 
{
    // aliases 
    using polygon2_implf = polygon2_impl<float>;
}

}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_level_polygon_common_impl_h_included

/*
The 3-Clause BSD License

Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.

Contributors: Sherry Ignatchenko, Volodymyr Melnychuk

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