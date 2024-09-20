/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_barycentric_impl_h_included
#define sixit_geometry_barycentric_impl_h_included

#include "sixit/geometry/barycentric.h"
#include "sixit/geometry/low_level/vector.h"

namespace sixit
{
namespace geometry
{
    template <typename fp>
    barycentric<fp>::barycentric(const point2<fp>& p, const triangle2<fp>& t)
    {
        auto v1 = t.p2().vec() - t.p3().vec();
        auto v2 = t.p1().vec() - t.p3().vec();

        auto det = v1.y * v2.x - v1.x * v2.y;

        auto v = p.vec() - t.p3().vec();

        l1 = (v1.y * v.x - v1.x * v.y) / det;
        l2 = (- v2.y * v.x + v2.x * v.y) / det;
        l3 = sixit::units::create_dimensionless_scalar<fp>(1.f) - l1 - l2;
    }

    template <typename fp>
    point2<fp> barycentric<fp>::point(const triangle2<fp>& t) const
    {
        return point2<fp>(t.p1().vec() * l1 + t.p2().vec() * l2 + t.p3().vec() * l3);
    }

    template <typename fp>
    point3<fp> barycentric<fp>::point(const triangle3<fp>& t) const
    {
        
        return point3<fp>(t.vertex(0).vec() * sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::simple_scalar>(l1) + 
                      t.vertex(1).vec() * sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::simple_scalar>(l2) + 
                      t.vertex(2).vec() * sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::simple_scalar>(l3));
    }
};
};

#endif //sixit_geometry_barycentric_impl_h_included

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