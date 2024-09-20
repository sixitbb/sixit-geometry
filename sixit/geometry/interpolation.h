/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_interpolation_h_included
#define sixit_geometry_interpolation_h_included

#include "sixit/geometry/low_level/interpolation_helper.h"
#include "sixit/geometry/sixit_mathf.h"

#include <tuple>
#include <vector>
#include <utility>

namespace sixit
{
namespace geometry
{

// mb: TODO interpolation_span and interpolation_equidistant_span
// have some kind of concept in common

template<typename T, typename fp>
class interpolation_span
{
    std::vector<std::pair<sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>, T>> key_values;
public:
    typedef T value_type;

    interpolation_span(std::vector<std::pair<sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>, T>> key_values) 
        : key_values(std::move(key_values))
        {
            if (this->key_values.size() < 2)
                throw std::runtime_error("too few values");
        }
    interpolation_span(std::vector<sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>> keys, std::vector<T> values)
    {
        auto count = std::min(keys.size(), values.size());
        if (count < 2)
            throw std::runtime_error("too few values");

        for (decltype(count) i = 0; i < count; ++i)
        {
            key_values.emplace_back(keys[i], values[i]);
        }
    }


    size_t size() const { return key_values.size(); }
    sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> key(size_t idx) const { return key_values.at(idx).first; }
    const T& value(size_t idx) const { return key_values.at(idx).second; }

    size_t rank(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> x) const
    {
        if(x < key(0) || x >= key(size() - 1))
            throw std::runtime_error("out of range");


        // binary search
        size_t l = 0;
        size_t r = size();
        while(l < r)
        {
            size_t m = (l + r) / 2;
            if (key(m) < x)
                l = m + 1;
            else
                r = m;
        }

        return l;
    }
};

template<typename T, typename fp>
class interpolation_equidistant_span
{
    sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> key0 = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero();
    sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> keyDistance = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero();
    std::vector<T> values;
public:
    typedef T value_type;

    interpolation_equidistant_span(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> key0, 
    sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> keyDistance, std::vector<T> values)
        : key0(key0), keyDistance(keyDistance), values(std::move(values))
        {
            if (this->values.size() < 2)
                throw std::runtime_error("too few values");
        }


    size_t size() const { return values.size(); }
    sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> key(size_t idx) const { 
        return key0 + keyDistance * sixit::units::create_dimensionless_scalar<fp>(float(idx));
    }
    const T& value(size_t idx) const { return values.at(idx); }

    size_t rank(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> x) const
    {
        if(x < key0 || x >= key(size() - 1))
            throw std::runtime_error("out of range");

        return static_cast<size_t>(
            sixit::geometry::low_level::mathf::fp2int(sixit::geometry::low_level::mathf::floor((x - key0) / keyDistance)));
    }

};

template<typename Span>
class lerp
{
    Span sp;

public:
    lerp(Span sp) : sp(std::move(sp)) {}

    template <typename fp>
    typename Span::value_type interpolate(fp x) const
    {
        size_t idx = 0;
        if (x < sp.key(0))
        {
            // extrapolate
            idx = 0;
        }
        else if(x >= sp.key(sp.size() - 1))
        {
            //extrapolate
            idx = sp.size() - 2;
        }
        else
        {
            idx = sp.rank(x);

        }

        return low_level::_lerp_helper<fp>(sp.key(idx), sp.value(idx), sp.key(idx + 1), sp.value(idx + 1), x);
    }
};


} // namespace geometry
} // namespace sixit


#endif // sixit_geometry_interpolation_h_included

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