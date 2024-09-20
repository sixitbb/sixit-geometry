/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_point3_cloud_h_included
#define sixit_geometry_point3_cloud_h_included

#include "sixit/geometry/point.h"
#include "sixit/geometry/plane.h"

#include <vector>
#include <utility>

namespace sixit
{
namespace geometry
{
    template <typename fp>
    struct point3_cloud
    {
        template<class PCloud> friend plane3<fp> best_fit_plane3(const PCloud& pcloud);

      private:
        std::vector<point3<fp>> points;

        inline point3_cloud(const std::vector<point3<fp>>& p): points{p} {}
        size_t get_size() const { return points.size(); }

        inline point3<fp>& get_point(size_t idx)
        {
            assert(idx < points.size());
            return points[idx];
        }
        inline const point3<fp>& get_point(size_t idx) const
        {
            assert(idx < points.size());
            return points[idx];
        }

      public:
        inline point3_cloud();

        inline point3_cloud(const point3_cloud&) = delete;
        inline point3_cloud(point3_cloud&& other) noexcept = default;

        inline point3_cloud& operator=(const point3_cloud& other) = delete;
        inline point3_cloud& operator=(point3_cloud&& other) noexcept = default;

        inline point3_cloud clone() const
        {
            return point3_cloud(points);
        }

        inline void add(const point3<fp>& p);
        inline point3<fp> centroid() const;

        bounds3<fp> bounds() const
        {
            auto bb = bounds3<fp>(points[0]);
            for (size_t i = 1; i < points.size(); ++i)
                bb.expand_by(points[i]);

            return bb;
        }

    };
}; // namespace geometry

namespace geometry 
{
    // aliases 
    using point3_cloudf = point3_cloud<float>;
}

}; // namespace sixit

#endif //sixit_geometry_point3_cloud_h_included
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