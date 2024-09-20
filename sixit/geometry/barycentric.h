/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_barycentric_h_included
#define sixit_geometry_barycentric_h_included

#include "sixit/geometry/line_segment.h"
#include "sixit/core/units.h"

namespace sixit
{
namespace geometry
{
    template <typename fp>
    struct barycentric
    {
    private:
        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> l1 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(0.f);
        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> l2 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(0.f);
        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> l3 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(0.f);
    public:
        inline barycentric(const point2<fp>& p, const triangle2<fp>& t);
        inline point2<fp> point(const triangle2<fp>& t) const;
        inline point3<fp> point(const triangle3<fp>& t) const;
    };
};

namespace geometry 
{
  using barycentricf = barycentric<float>;
}

};

#endif //sixit_geometry_barycentric_h_included

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