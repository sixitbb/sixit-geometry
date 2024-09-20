/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_indexed_point3_cloud_impl_h_included
#define sixit_geometry_indexed_point3_cloud_impl_h_included

#include "sixit/geometry/indexed_point3_cloud.h"

namespace sixit
{
namespace geometry
{
	template <typename fp>
	indexed_point3_cloud<fp>::indexed_point3_cloud(const low_level::reference_container<point3<fp>>& ref, const std::vector<size_t>& _indices)
		: ref_container(ref)
		, indices(_indices)
	{

	}

	template <typename fp>
	void indexed_point3_cloud<fp>::add(size_t index)
	{
		indices.push_back(index);
	}

	template <typename fp>
	point3<fp> indexed_point3_cloud<fp>::centroid() const
	{
		low_level::dimensional_vector3<fp, sixit::units::length_unit::dim> sum(low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::zero());
		for (size_t i = 0; i < indices.size(); ++i)
		{
			auto p = get_point(i);
			sum += p.vector;
		}
		sum /= sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(fp(float(indices.size())));
		return point3<fp>(sum);
	}
}
};

#endif //sixit_geometry_indexed_point3_cloud_impl_h_included

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