/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/
    
#ifndef sixit_geometry_indexed_point3_cloud_h_included
#define sixit_geometry_indexed_point3_cloud_h_included
    
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/generic_reference.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/line.h"

#include "sixit/core/lwa.h"
#include <vector>

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
    struct indexed_point3_cloud
    {
        friend struct  sixit::lwa::fmt::formatter<sixit::geometry::indexed_point3_cloud<fp>>;
        template<class PCloud> friend plane3<fp> best_fit_plane3(const PCloud& pcloud);
        friend imp_scene_to_mesh3_converter;
        friend R3DSystem_Designer::imp_scene_to_mesh3_converter;

      private:
        const low_level::reference_container<point3<fp>>& ref_container;
        std::vector<size_t> indices;

        inline indexed_point3_cloud(const low_level::reference_container<point3<fp>>& ref, const std::vector<size_t>& indices);
        inline indexed_point3_cloud(const indexed_point3_cloud&) = delete;
        inline indexed_point3_cloud(indexed_point3_cloud &&other) noexcept
            : ref_container(other.ref_container), indices(std::move(other.indices)) {}

        indexed_point3_cloud clone() const
        {
            return indexed_point3_cloud(ref_container, indices);
        }

        inline const point3<fp>& get_point(size_t idx) const
        {
            assert(idx < indices.size());
            return ref_container[indices[idx]];
        }
        size_t get_size() const { return indices.size(); }

        bounds3<fp> bounds() const
        {
            auto bb = bounds3<fp>(ref_container[indices[0]]);
            for (size_t i = 1; i < indices.size(); ++i)
                bb.expand_by(ref_container[indices[i]]);

            return bb;
        }

      public:
        inline void add(size_t index);
        inline point3<fp> centroid() const;
    };
};  // namespace geometry

namespace geometry 
{
    // aliases 
    using indexed_point3_cloudf = indexed_point3_cloud<float>;
}

};  // namespace sixit

#endif //sixit_geometry_indexed_point3_cloud_h_included

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