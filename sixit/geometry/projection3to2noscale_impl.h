/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_projection3to2noscale_impl_h_included
#define sixit_geometry_projection3to2noscale_impl_h_included

#include "sixit/geometry/projection3to2noscale.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/transforms.h"
#include "sixit/geometry/projection.h"

namespace sixit
{
	namespace geometry
	{
		template <typename fp>
		projection3to2noscale<fp>::projection3to2noscale(const triangle3<fp>& tri)
			:projection3to2noscale(tri.plane())
		{
			
		}

		template <typename fp>
		projection3to2noscale<fp>::projection3to2noscale(const plane3<fp>& pl) :plane(pl)
		{
		    fp cx = sixit::geometry::low_level::mathf::abs(plane.normal.vec().x());
		    fp cy = sixit::geometry::low_level::mathf::abs(plane.normal.vec().y());
		    fp cz = sixit::geometry::low_level::mathf::abs(plane.normal.vec().z());
		    set_component_map(cx, cy, cz);
		}
	    
		template <typename fp>
	    inline projection3to2noscale<fp>::projection3to2noscale(const plane3<fp>& pl, int set_dir) : plane(pl)
		{
		    fp cx = set_dir == 0 ? 0 : abs(plane.normal.vec().x());
		    fp cy = set_dir == 1 ? 0 : abs(plane.normal.vec().y());
		    fp cz = set_dir == 2 ? 0 : abs(plane.normal.vec().z());
            set_component_map(cx, cy, cz);
		}

		template <typename fp>
		point2<fp> projection3to2noscale<fp>::transform(const point3<fp>& p) const
		{
			return point2(sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec()[map[0]]), 
							sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(p.vec()[map[1]]));
		}

		template <typename fp>
		template <typename T>
		inline typename T::projectionT projection3to2noscale<fp>::transform(const T& o) const
		{
			using vlo = std::vector<point2<fp>>;
			std::vector<point3<fp>> points3_span = o.template _get_projection_points<std::vector<point3<fp>>>();

			vlo points2_span;
			points2_span.reserve(points3_span.size());

			for (size_t i = 0; i < points3_span.size(); ++i)
			{
				points2_span.push_back(transform(points3_span[i]));
			}

			return typename T::projectionT(points2_span);
		}

		template <typename fp>
		template<template<typename> class VLO>
		SIXIT_LWA_OPTIONAL_REQUIRES_FP(sixit::geometry::reference_points3, VLO<point3<fp>>, fp)
		VLO<point2<fp>> projection3to2noscale<fp>::transform(const VLO<point3<fp>>& pnts)
		{
			std::vector<point2<fp>> pts2d;//no means in ref points to add elements in it, should do this way
			pts2d.reserve(pnts.get_size());
			for (auto i = 0; i < pnts.get_size(); ++i)
			{
				pts2d.push_back(transform(pnts[i]));
			}
			return VLO<point2<fp>>(pts2d);
		}

		// TEMPORALY SOLUTION MUST BE DELETED AND REPLACED WITH TEMPLATED ONE
		template <typename fp>
        sixit::geometry::low_level::reference_container<point2<fp>> projection3to2noscale<fp>::transform(const sixit::geometry::low_level::reference_container<point3<fp>>& pnts)
		{
			std::vector<point2<fp>> pts2d;//no means in ref points to add elements in it, should do this way
			pts2d.reserve(pnts.get_size());
			for (auto i = 0; i < pnts.get_size(); ++i)
			{
				pts2d.push_back(transform(pnts[i]));
			}
			return sixit::geometry::low_level::reference_container<point2<fp>>(pts2d);
		}

		template <typename fp>
	    inline void projection3to2noscale<fp>::set_component_map(fp cx, fp cy, fp cz)
	    {
		    if (cx >= cy && cx >= cz)
		    {
		        map[0] = 1;
		        map[1] = 2;
		    }
		    else if (cy >= cx && cy >= cz)
		    {
		        map[0] = 0;
		        map[1] = 2;
		    }
		    else
		    {
		        map[0] = 0;
		        map[1] = 1;
		    }
	    }

		template <typename fp>
		inline affine_transform3<fp> projection3to2noscale<fp>::affine_transform() const
		{
			low_level::matrix4x4 m = low_level::matrix4x4<fp>::identity();
			// Dropping the most orthogonal coordinate for a "convenient" coordinate system
			if (map[0] != 0)
				m[0] = { 0,0,0, 1 };    // drop x
			else if (map[1] == 2)
				m[1] = { 0,0,0, 1 };    // drop y
			else
				m[2] = { 0,0,0, 1 };    // drop z

			return geometry::affine_transform3(m);
		}
};
};

#endif //sixit_geometry_projection3to2noscale_impl_h_included

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