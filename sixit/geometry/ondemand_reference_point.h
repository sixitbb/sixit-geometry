/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_ondemand_reference_point_h_included
#define sixit_geometry_ondemand_reference_point_h_included

#include "reference_concepts.h"
#include <memory>
#include <unordered_map>

namespace sixit
{
namespace geometry
{
    template <typename fp>
	struct point3;
    template <typename fp>
	struct point2;

    template<template<class> class base_refpoints, class point_type, class transform, class cache = std::unordered_map<unsigned, typename base_refpoints<point_type>::point_type>>
    class ondemand_reference_points
    {
    public:
        template<template<class> class base_refpoints0, class point_type0, class transform0>
        ondemand_reference_points(const base_refpoints0<point_type0>* ref, const transform0& transform1)
            : ref_points_(ref), transform_(transform1) {}

        inline point_type operator[](unsigned i)
        {
            if (cache_.find(i) != cache_.end())
                return cache_[i];

            point_type transformedPoint = transform_.transform((*ref_points_)[i]);
            cache_[i] = transformedPoint;
            return transformedPoint;
        }

    private:
        const base_refpoints<point_type>* ref_points_;
        transform transform_;
        cache cache_;
    };
    
}
}

#endif //sixit_geometry_ondemand_reference_point_h_included

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