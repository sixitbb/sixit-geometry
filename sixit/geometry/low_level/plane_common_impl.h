/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_plane_common_impl_h_included
#define sixit_geometry_low_level_plane_common_impl_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/rotation.h"

#include <utility>

namespace sixit
{
namespace geometry
{
namespace low_level
{
    template <typename fp>
    struct plane3_impl
    {
        inline static std::optional<point3<fp>> intersection(const direction3<fp>& normal, const point3<fp>& point, const point3<fp>& p1, const point3<fp>& p2)
        {
            low_level::dimensional_vector3 line_dir = (p2.vec() - p1.vec()).normalized();
            auto dot = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::dot(line_dir, normal.vec());
            auto t = low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::dot(point.vec() - p1.vec(), normal.vec()) / dot;
            if (sixit::geometry::low_level::mathf::isfinite(t))
                return point3(p1.vec() + (line_dir * t) * sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.f));
            else
                return std::nullopt;
        }
    };
}; // namespace low_level

namespace low_level 
{
    // aliases 
    using plane3_implf = plane3_impl<float>;
}

}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_level_plane_common_impl_h_included

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