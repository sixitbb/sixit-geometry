/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Serhii Iliukhin
*/

#ifndef sixit_geometry_sixit_mathf_h_included
#define sixit_geometry_sixit_mathf_h_included

#include <limits>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <assert.h>

#include <sixit/core/lwa.h>
#include "sixit/core/units.h"
#include <sixit/dmath/traits.h>
#include <sixit/core/lwa.h>

#include "sixit/dmath/mathf/mathf.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{
namespace mathf
{
    template<typename fp>
    inline fp make_constant_from(float f)
    {
        if constexpr (std::is_base_of_v<sixit::units::dimensional_scalar_base, fp>)
        {
            static_assert(fp::dim.is_dimensionless());
            typename fp::fp fp_val(f);
            return fp({ fp_val, sixit::units::internal_constructor_of_dimensional_scalar_from_fp() });
        }
        else
        {
            return f;
        }
    }

    template<typename fp, sixit::lwa::floating_const_helper f>
    inline constexpr auto make_constant_from()
    {
        if constexpr (std::is_base_of_v<sixit::units::dimensional_scalar_base, fp>)
        {
            //static_assert(fp::dim.is_dimensionless());
            constexpr typename fp::fp fp_val(f.val());
            return fp({ fp_val, sixit::units::internal_constructor_of_dimensional_scalar_from_fp() });
        }
        else
        {
            return f.val();
        }
    }

    template<typename fp>
    inline constexpr fp make_zero()
    {
        if constexpr (std::is_base_of_v<sixit::units::dimensional_scalar_base, fp>)
            return fp::zero();
        else
            return 0.f;
    }
    
    template <typename fp> 
    inline int fp2int(fp val) {
        return int(sixit::dmath::fp_traits<fp>::fp2int64(val));
    }

    template <typename fp, sixit::units::physical_dimension dim>
    inline int fp2int(sixit::units::dimensional_scalar<fp, dim> val) {
        return int(sixit::dmath::fp_traits<fp>::fp2int64(val.value));
    }
       
    static constexpr float pi = (float)3.141592653589793;
    static constexpr float deg_2_rad = pi / 180.0f; 
    static constexpr float rad_2_deg = 180.0f / pi; 
    static constexpr float epsilon = std::numeric_limits<float>::epsilon();
    static constexpr float infinity = std::numeric_limits<float>::infinity();
    static constexpr float max_float = std::numeric_limits<float>::max();
    static constexpr float min_float = std::numeric_limits<float>::min();
    static constexpr float negative_infinity = -infinity;

    template<typename fp, const float s_members[], int s_size>
    fp taylor_sequence(fp value, fp mvalue) {
        fp fp_0(0.f);
        fp rv = fp_0;
        for (size_t i = 0; i < s_size; i++)
        {
            rv += value * fp(s_members[i]);
            value *= mvalue;
        }
        return rv;
    }

    template<typename fp>
    inline fp abs(fp val) { return sixit::dmath::mathf::abs(val); }

    template <typename fp> 
    inline auto ceil(fp val) { return sixit::dmath::mathf::ceil(val); }

    template <typename fp> 
    inline bool isfinite(fp val) { return sixit::dmath::mathf::isfinite(val); }

    template <typename fp>
    inline int ceil_to_int(fp val)
    {
        return fp2int(sixit::dmath::fp_traits<fp>::ceil(val));
    }

    template <typename fp> 
    inline auto floor(fp val) { return sixit::dmath::mathf::floor(val); }

    template <typename fp>
    inline int floor_to_int(fp val)
    {
        return fp2int<fp>(floor<fp>(val));
    }

    template <typename fp>
    inline fp min(fp a, fp b) { return sixit::dmath::mathf::min(a, b); }

    template <typename fp>
    inline fp max(fp a, fp b) { return sixit::dmath::mathf::max(a, b); }

    template<typename fp>
    inline fp clamp(fp val, fp min_, fp max_)
    {
        return max(min_, min(val, max_));
    }

    template<typename fp>
    inline fp clamp01(fp val)
    {
        fp fp_0 = make_constant_from <fp, 0.f>();
        fp fp_1 = make_constant_from <fp, 1.f>();
        return clamp(val, fp_0, fp_1);
    }

    inline int next_power_of_two(int val)
    {
        --val;
        val |= val >> 1;
        val |= val >> 2;
        val |= val >> 4;
        val |= val >> 8;
        val |= val >> 16;
        return val + 1;
    }

    inline int closest_power_of_two(int val)
    {
        int next = next_power_of_two(val);
        int prev = next >> 1;
        return next - val < val - prev ? next : prev;
    }

    template <typename fp>
    inline auto trunc(fp val) { return sixit::dmath::mathf::trunc(val); }

    template <typename fp>
    inline auto fmod(fp val, fp max) { return sixit::dmath::mathf::fmod(val, max); }

    inline bool is_power_of_two(int val)
    {
        if (val == 0)
        {
            return false;
        }
        return (val & (val - 1)) == 0;
    }
 
    template <typename fp>
    inline auto round(fp val) { return sixit::dmath::mathf::round(val); }
    
    template <typename fp>
    inline int round_to_int(fp val)
    {
        return fp2int(round(val));
    }
    
    template <typename fp>
    inline int8_t sign(fp val) { return sixit::dmath::mathf::sign(val); }

    template <typename fp>
    inline auto sin(fp x) { return sixit::dmath::mathf::sin(x); }

    template <typename fp>
    inline auto cos(fp x) { return sixit::dmath::mathf::cos(x); }

    template <typename fp>
    inline auto sqrt(fp x) { return sixit::dmath::mathf::sqrt(x); }

    template <typename fp>
    inline auto tan(fp x) { return sixit::dmath::mathf::tan(x); }

    template <typename fp>
    inline auto acos(fp x) { return sixit::dmath::mathf::acos(x); }

    template <typename fp>
    inline auto asin(fp x) { return sixit::dmath::mathf::asin(x); }
    
    template <typename fp>
    inline auto atan(fp x) { return sixit::dmath::mathf::atan(x); }

    template <typename fp>
    inline auto atan2(fp y, fp x) { return sixit::dmath::mathf::atan2(y, x); }

    template <typename fp>
    inline auto exp(fp x) { return sixit::dmath::mathf::exp(x); }

    template <typename fp>
    inline auto log(fp x) { return sixit::dmath::mathf::log(x); }

    template <typename fp>
    inline auto log10(fp x) { return sixit::dmath::mathf::log10(x); }
}; // namespace mathf
}; // namespace low_level
}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_sixit_mathf_h_included
/*
The 3-Clause BSD License

Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.

Contributors: Sherry Ignatchenko, Serhii Iliukhin

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