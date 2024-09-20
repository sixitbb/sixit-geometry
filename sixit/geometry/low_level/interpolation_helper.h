/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_interpolation_helper_h_included
#define sixit_geometry_low_level_interpolation_helper_h_included

#include <tuple>
#include <utility>

namespace sixit
{
namespace geometry
{
namespace low_level
{

constexpr class from_interpolation_tag_t
{

} from_interpolation_tag{};

class interpolation_friend
{
  public:
    template<typename T>
    static auto get_interpolation_data(const T& t)
    {
        static_assert(std::is_same_v<T, decltype(make<T>(t.get_interpolation_data()))>, 
            "type T() shoutd be constructible from its interpolation data using corresponding interpolation_friend::make()");

        return t.get_interpolation_data();
    }

    template<typename V, typename... T>
    static V make(const std::tuple<T...>& t)
    {
        return V{from_interpolation_tag, t};
    }
};

// forward-decl
template <typename fp, typename T>
T _lerp_helper(fp x0, const T& y0, fp x1, const T& y1, fp x);

template <typename fp>
fp _lerp_helper(fp x0, const fp& y0, fp x1, const fp& y1, fp x);

template<size_t N, typename... T, typename fp>
auto _lerp_helper_tuple(fp x0, const std::tuple<T...>& y0, fp x1, const std::tuple<T...>& y1, fp x)
{
    using type_n = typename std::tuple_element<N, std::tuple<T...>>::type;
    std::tuple<type_n> elem_n{_lerp_helper(x0, std::get<N>(y0), x1, std::get<N>(y1), x)};
    
    if constexpr(N + 1 == std::tuple_size<std::tuple<T...>>::value)
        return elem_n;
    else
        return std::tuple_cat(elem_n, _lerp_helper_tuple<N + 1>(x0, y0, x1, y1, x));
}

template <typename fp, typename T>
T _lerp_helper(fp x0, const T& y0, fp x1, const T& y1, fp x)
{
    return low_level::interpolation_friend::make<T>(_lerp_helper_tuple<0>(x0, 
        low_level::interpolation_friend::get_interpolation_data(y0), x1,
        low_level::interpolation_friend::get_interpolation_data(y1), x)
    );
}

template <typename fp>
fp _lerp_helper(fp x0, const fp& y0, fp x1, const fp& y1, fp x)
{
    return ((x - x0) * ((y1 - y0) / (x1 - x0))) + y0;
}

} // namespace low_level
} // namespace geometry
} // namespace sixit


#endif // sixit_geometry_low_level_interpolation_helper_h_included

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