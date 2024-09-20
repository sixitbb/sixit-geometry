/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_projection3to2noscale_h_included
#define sixit_geometry_projection3to2noscale_h_included

#include "sixit/geometry/point.h"
#include "sixit/geometry/line_segment.h"
#include "sixit/geometry/plane.h"
#include "sixit/core/lwa.h"

namespace sixit
{
namespace geometry
{
    template <typename fp>
    struct affine_transform3;

    template <typename fp>
    struct projection3to2noscale
    {
      private:
        plane3<fp> plane;
        int map[2] = { 0,0 };         // indices of the components to keep

        void set_component_map(fp cx, fp cy, fp cz);

      public:
        projection3to2noscale() : plane(direction3<fp>(), point3<fp>()) {}
        inline projection3to2noscale(const triangle3<fp> &tri);
        inline projection3to2noscale(const plane3<fp>& pl);
        inline projection3to2noscale(const plane3<fp>& pl, int set_dir);
        
        inline point2<fp> transform(const point3<fp>& p) const;

        template <typename T>
        inline typename T::projectionT transform(const T& o) const;

        template<template<typename> class VLO>
        SIXIT_LWA_OPTIONAL_REQUIRES_FP(sixit::geometry::reference_points3, VLO<point3<fp>>, fp)
        inline VLO<point2<fp>> transform(const VLO<point3<fp>>& p);

      // TEMPORALY SOLUTION MUST BE DELETED AND REPLACED WITH TEMPLATED ONE
        sixit::geometry::low_level::reference_container<point2<fp>> transform(const sixit::geometry::low_level::reference_container<point3<fp>>& pnts);

        inline affine_transform3<fp> affine_transform() const;

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"projection3to2noscale">(comparser, obj);
            sixit::rw::read_write<&projection3to2noscale::plane, "plane", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "map0", obj.map[0]);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "map1", obj.map[1]);
            sixit::rw::end_struct(comparser);
        }        
    };
}; // namespace geometry

namespace geometry 
{
    // aliases 
    using projection3to2noscalef = projection3to2noscale<float>;
}

}; // namespace sixit

#endif //sixit_geometry_projection3to2noscale_h_included

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
