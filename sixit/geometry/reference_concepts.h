/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/
#ifndef sixit_geometry_reference_concepts_h_included
#define sixit_geometry_reference_concepts_h_included

#include "sixit/core/lwa.h"

#include <concepts>
#include <functional>

namespace sixit
{
namespace graphics
{
template <class T>
class full_mesh3;

#if SIXIT_LWA_OPTIONAL_CONCEPT
template <typename T>
concept texture2 =
    requires(T t, const T tc, unsigned w, unsigned h, unsigned format, const void* data, unsigned y, uint8_t r,
             uint8_t g, uint8_t b, uint8_t a, uint32_t v, std::function<uint32_t(uint32_t)> f, typename T::rect rect) {
        {
            tc.get_width()
        } -> std::same_as<uint32_t>;
        {
            tc.get_height()
        } -> std::same_as<uint32_t>;
        {
            tc.get_format()
        } -> std::same_as<uint32_t>;
        {
            tc.gpu_buffer_size()
        } -> std::same_as<size_t>;
        {
            tc.is_loaded()
        } -> std::same_as<bool>;
        {
            tc.clone()
        } -> std::same_as<T>;
        {
            t.load(w, h, format, data)
        } -> std::same_as<void>;
        {
            t.load(w, h, format)
        } -> std::same_as<void>;
        {
            t.get_line(y)
        } -> std::same_as<uint32_t*>;
        {
            tc.get_line(y)
        } -> std::same_as<const uint32_t*>;
        {
            T::to_rgba(r, g, b, a)
        } -> std::same_as<uint32_t>;
        {
            T::to_rgba(r, g, b)
        } -> std::same_as<uint32_t>;
        {
            T::from_rgba(v)
        } -> std::same_as<std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>>;
        {
            t.adjust_pixels(f, rect)
        } -> std::same_as<void>;
        {
            t.adjust_pixels(f)
        } -> std::same_as<void>;
    };
#endif // SIXIT_LWA_OPTIONAL_CONCEPT
}
namespace geometry
{

#if SIXIT_LWA_OPTIONAL_CONCEPT
template <typename Callable>
concept callback_xy = requires (Callable fn, int x, int y)
{
    { fn(x, y) };
};

template <typename Callable>
concept callback_xyz = requires (Callable fn, int x, int y, int z)
{
    { fn(x, y, z) };
};
namespace internal
{
    template <typename T> constexpr static bool is_full_mesh_v = false;
    template <typename T> constexpr static bool is_full_mesh_v<sixit::graphics::full_mesh3<T>> = true;
}

template <typename T>
concept full_mesh_concept = internal::is_full_mesh_v<T>;

template <typename T>
concept mesh_buffers = requires (T b)
{
    typename T::Material;
    typename T::Vertex;
    typename T::Texcoord;
    typename T::Normal;
    typename T::Tangent;
    typename T::Bone;
    typename T::Index;
    typename T::BoneInfo;
};

#else
#pragma message("SIXIT_WARNING: SIXIT_LWA_OPTIONAL_CONCEPT is not defined, geometry concepts are not available")
#endif // SIXIT_LWA_OPTIONAL_CONCEPT

template <typename fp>
struct point3;
template <typename fp>
struct point2;

template<typename T>
concept get_size_concept = requires(const T& rp)
{
    { rp.get_size() } -> std::same_as<int64_t>;
};

template<typename T, typename fp>
concept reference_points3_nonconst = requires(const T& rp, size_t idx)
{
    { rp[idx] } -> std::same_as<point3<fp>>;
};

template<typename T, typename fp>
concept reference_points3_constref = requires(const T& rp, size_t idx)
{
    { rp[idx] } -> std::same_as<const point3<fp>&>;
};

template<typename T, typename fp>
concept reference_points3 = get_size_concept<T> && (reference_points3_nonconst<T, fp> || reference_points3_constref<T, fp>);

template<typename T, typename fp>
concept reference_points2_nonconst = requires(const T& rp, size_t idx)
{
    { rp[idx] } -> std::same_as<point2<fp>>;
};

template<typename T, typename fp>
concept reference_points2_constref = requires(const T& rp, size_t idx)
{
    { rp[idx] } -> std::same_as<const point2<fp>&>;
};

template<typename T, typename fp>
concept reference_points2 = get_size_concept<T> && (reference_points2_nonconst<T, fp> || reference_points2_constref<T, fp>);

}
}

#endif //sixit_geometry_reference_concepts_h_included

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