/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_plane_h_included
#define sixit_geometry_plane_h_included

#include <optional>
#include <tuple>

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/low_level/plane_common_impl.h"

namespace sixit
{
namespace geometry
{
    template <typename fp>
    struct plane3
    {
        friend struct  sixit::lwa::fmt::formatter<plane3<fp>>;
        friend struct low_level::tri3_impl<fp>;
        friend struct projection3<fp>;
        friend struct projection3to2<fp>;
        friend struct projection3to2ex<fp>;
        friend struct projection3to2noscale<fp>;
        friend struct mirror_transform3<fp>;
        friend struct triangle3<fp>;
        template <typename fp1>
        friend point3<fp1> _orthogonal_projection_helper(const point3<fp1>& p, const plane3<fp1>& plane);
        friend class voxelizer;
        friend sixit::graphics::intersectable_mesh;

      private:
        direction3<fp> normal;
        point3<fp> point;
        inline std::tuple<fp, fp, fp, fp> abcd() const;

      public:
        inline plane3(const direction3<fp>& normal, const point3<fp>& point);

        inline sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> distance(const point3<fp>& p) const;
        inline bool side(const point3<fp>& p) const;

        inline plane3<fp> orthogonal_plane(const line3<fp>& line) const;
        inline line3<fp> orthogonal_line(const point3<fp>& point) const;

        inline bounds3<fp> bounds() const
        {
            return bounds3<fp>(point);
        }

        inline std::optional<line3<fp>> intersection(const plane3<fp>& o) const;

        template<typename LS3>
        inline std::optional<point3<fp>> intersection(const LS3& ls) const
        {
            return low_level::plane3_impl<fp>::intersection(normal, point, ls.p1(), ls.p2());
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"plane3">(comparser, obj);
            sixit::rw::read_write<&plane3::normal, "normal", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&plane3::point, "point", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

   // template<typename VLO> plane3 best_fit_plane3(const VLO& vlo);

}; // namespace geometry

namespace geometry 
{
    // aliases 
    using plane3f = plane3<float>;
}


}; // namespace sixit

#endif //sixit_geometry_plane_h_included
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
