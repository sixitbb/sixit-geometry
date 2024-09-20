/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_tritriintersect_h_included
#define sixit_geometry_tritriintersect_h_included

#include <optional>
#include <vector>

#include "sixit/core/lwa.h"
#include "sixit/geometry/line_segment.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/projection2to1noscale.h"
#include "sixit/geometry/projection3to2noscale.h"

namespace sixit
{
namespace geometry
{
    template <typename fp>
    class triTriIntersect
    {
      private:
        std::vector<point2<fp>> intersections(point3<fp> p0, point3<fp> p1, point3<fp> p2, projection3to2noscale<fp> proj, line2<fp> inter_line)
        {
            std::vector<point2<fp>> inter_points;
            line_segment2<fp> edges[3] = {
                line_segment2<fp> ( proj.transform(p0), proj.transform(p1) ),
                line_segment2<fp> ( proj.transform(p1), proj.transform(p2) ),
                line_segment2<fp> ( proj.transform(p2), proj.transform(p0) )
            };

            for (const auto& edge : edges)
            {
                std::optional<point2<fp>> inter_point = inter_line.strict_intersection(edge);
                if (inter_point)
                    inter_points.push_back(*inter_point);
            }

            return inter_points;
        }

      public:
        bool intersects(point3<fp> a0, point3<fp> a1, point3<fp> a2, point3<fp> b0, point3<fp> b1, point3<fp> b2)
        {
            triangle3<fp> t3(a0, a1, a2);
            triangle3<fp> ot3(b0, b1, b2);
            
            projection3to2noscale<fp> proj32_ta ( t3.plane());

            std::optional<line3<fp>> intersection = t3.plane().intersection(ot3.plane());

            if (!intersection)
            {
                sixit::lwa::fmt::print("parallel planes\n");
                return false;
            }

            line3<fp> inter_line = *intersection;
            line2<fp> inter_line_2d = proj32_ta.transform(inter_line);

            std::vector<point2<fp>> inter_points_a = intersections(a0, a1, a2, proj32_ta, inter_line_2d);
            if (inter_points_a.size() < 2) return false;
            std::vector<point2<fp>> inter_points_b = intersections(b0, b1, b2, proj32_ta, inter_line_2d);
            if (inter_points_b.size() < 2) return false;

            // project segments to 1D
            projection2to1noscale<fp> proj21 ( line_segment2<fp> (inter_points_a[0], inter_points_a[1]) );
            line_segment1<fp> s1a ( proj21.transform( inter_points_a[0] ), proj21.transform( inter_points_a[1] ) );
            line_segment1<fp> s1b ( proj21.transform( inter_points_b[0] ), proj21.transform( inter_points_b[1] ) );

            return s1a.overlays(s1b);
        }
    };

}; // namespace geometry

namespace geometry 
{
  // aliases
  using triTriIntersectf = triTriIntersect<float>;
}; // namespace geometry

}; // namespace sixit

#endif //sixit_geometry_tritriintersect_h_included

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