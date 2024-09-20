/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_rotation_h_included
#define sixit_geometry_rotation_h_included

//#define TRS_AS_MATRIX4X4
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/quaternion.h"
#include "sixit/geometry/low_level/complex_number_rotation.h"
#include "sixit/geometry/units.h"
#include "sixit/geometry/point.h"
#include "sixit/geometry/_internal/simd/mat4.h"
#include "sixit/geometry/_internal/simd/quat4.h"
#include "sixit/geometry/low_level/matrix.h"

#include <tuple>

namespace sixit
{

// TODO: Move to some common geometry header
// This forward declaration is needed to avoid including rw headers
namespace rw
{
struct constructor_for_rw_tag;
struct struct_default_value_type;
namespace comparsers {
    using constructor_for_rw_tag = rw::constructor_for_rw_tag;
    using StructDefaultValueType = rw::struct_default_value_type;
}
}

namespace geometry
{
    template <typename fp>
    struct rotation3
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::rotation3<fp>>;
        friend struct trs3<fp>;
        friend struct affine_transform3<fp>;
        friend struct affine_transform2<fp>;
        friend struct projection3to2ex<fp>;
        friend struct graphics::trs3_animation;
        friend class graphics::skeleton_joint;
        template <typename fp1> friend rotation3<fp1> low_level::for_renderers_only::make_rotation3_dont_use_outside_of_renderer(fp1 x, fp1 y, fp1 z, fp1 w);
        template <typename fp1> friend rotation3<fp1> low_level::for_importer_only::make_rotation3_dont_use_outside_of_importer(fp1 x, fp1 y, fp1 z, fp1 w);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> low_level::for_renderers_only::xyzw_dont_use_outside_of_renderer(const rotation3<fp1>& r);
        template <typename fp1> friend gpu::mat4<fp1> low_level::for_renderers_only::matrix_dont_use_outside_of_renderer(const trs3<fp1>& trs);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> low_level::for_importer_only::xyzw_dont_use_outside_of_importer(const rotation3<fp1>& rt3);
        template <typename fp1> friend rotation3<fp1> low_level::for_data_transfers_only::make_rotation3_dont_use_outside_of_data_transfers(fp1 x, fp1 y, fp1 z, fp1 w);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> low_level::for_data_transfers_only::xyzw_dont_use_outside_of_data_transfers(const rotation3<fp1>& r);
      private:
        low_level::dimensional_quaternion<fp, sixit::units::simple_scalar::dim> q;
        inline low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> euler_angles() const;

      public:
        inline rotation3(const direction3<fp>& v1, const direction3<fp>& v2);
        inline rotation3();
		    inline rotation3(const direction3<fp>& axis, radians<fp> angle);
        inline rotation3(radians<fp> roll, radians<fp> pitch, radians<fp> yaw);
        inline rotation3(degrees<fp> roll, degrees<fp> pitch, degrees<fp> yaw);
        inline rotation3(axis_x_tag tag, radians<fp> angle);
        inline rotation3(axis_y_tag tag, radians<fp> angle);
        inline rotation3(axis_z_tag tag, radians<fp> angle);
        inline rotation3<fp> opposite() const;
        inline direction3<fp> rotate(const direction3<fp>& dir) const;
        inline void flip();
        inline rotation3 operator+(const rotation3& o) const;
        inline rotation3& operator+=(const rotation3& o);

        bool operator==(const rotation3& other) const
        {
            return (q.x() == other.q.x()) && (q.y() == other.q.y()) && (q.z() == other.q.z()) && (q.w() == other.q.w());
        }

        [[nodiscard]] inline bool _for_test_only_approximate_eq(const rotation3& other, size_t n = 1) const
        {
            return q._for_test_only_approximate_eq(other.q, n);
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            decltype(obj.q)::read_write(obj.q, comparser);
        }
    };

    template <typename fp>
    struct rotation2
    {
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::rotation2<fp>>;
        friend struct low_level::ls2_impl<fp>;
        friend struct trs2<fp>;

      private:
        //low_level::quaternion q;
        low_level::complex_number_rotation<fp> q;
        inline sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> euler_angle() const;

      public:
        inline rotation2(const direction2<fp>& v1, const direction2<fp>& v2);
        inline rotation2();
        inline rotation2 opposite() const;
        inline void flip();
        inline rotation2 operator+(const rotation2<fp>& o) const;
        inline rotation2& operator+=(const rotation2<fp>& o);

        inline static rotation2 right_angle();
        inline static rotation2 straight_angle();
        inline rotation2 orthogonal() const;
        inline direction2<fp> rotate(const direction2<fp>& dir) const;
    };

    template <typename fp>
    struct rotation1
    {
        // friend struct sixit::lwa::fmt::formatter<sixit::geometry::rotation1>;
      private:
        bool rotation;
        inline fp euler_angle() const;
        inline rotation1(bool rot): rotation(rot) {}

      public:
        inline rotation1(const direction1<fp>& v1, const direction1<fp>& v2);
        inline rotation1(): rotation(true) {}
        inline rotation1 opposite() const;
        inline void flip();
        inline rotation1 operator+(const rotation1<fp>& o) const;
        inline rotation1& operator+=(const rotation1<fp>& o);

        inline static rotation1 right_angle();
        inline static rotation1 left_angle();
    };

    template <typename fp>
    struct translation3
    {
        friend struct trs3<fp>;
        friend struct affine_transform3<fp>;
        friend struct graphics::trs3_animation;
        friend class graphics::skeleton_joint;
        friend class graphics::bone_formatter;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::translation3<fp>>;
        template <typename fp1>
        friend gpu::mat4<fp1> low_level::for_renderers_only::matrix_dont_use_outside_of_renderer(const trs3<fp1>& trs);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_importer_only::xyz_dont_use_outside_of_importer(const geometry::translation3<fp1>& tr3);
      private:
        low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> vec;

      public:
        // constructor for parsers
        translation3(const rw::comparsers::constructor_for_rw_tag&)
        {
        }
        inline translation3(const point3<fp>& v): vec(v.vec()) {};

        bool operator==(const translation3<fp>& other) const
        {
            return vec == other.vec;
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"translation3">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "x", obj.vec.x());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "y", obj.vec.y());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "z", obj.vec.z());
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct translation2
    {
        friend struct trs2<fp>;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::translation2<fp>>;

      private:
        low_level::vector2<fp> vec;

      public:
        inline translation2(const point2<fp>& v): vec(v.vec()) {};

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"translation2">(comparser, obj);
            sixit::rw::read_write<&translation2::vec, "vec", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct scale3
    {
        friend struct trs3<fp>;
        friend struct affine_transform3<fp>;
        friend struct graphics::trs3_animation;
        friend class graphics::skeleton_joint;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::scale3<fp>>;
        template <typename fp1>
        friend gpu::mat4<fp1> low_level::for_renderers_only::matrix_dont_use_outside_of_renderer(const trs3<fp1>& trs);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1>  low_level::for_importer_only::xyz_dont_use_outside_of_importer(const geometry::scale3<fp1>& sc3);

      private:
        low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> vec;
        public:
        inline scale3(const point3<fp>& v): vec(v.vec()) {};
        inline scale3(): vec(
                        sixit::units::create_dimensionless_scalar<fp>(1.0f), 
                        sixit::units::create_dimensionless_scalar<fp>(1.0f), 
                        sixit::units::create_dimensionless_scalar<fp>(1.0f)) {};
        inline scale3(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>  s): vec(s, s, s) {};

        bool operator==(const scale3& other) const
        {
            return vec == other.vec;
        }

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"scale3">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "x", obj.vec.x());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "y", obj.vec.y());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "z", obj.vec.z());
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct scale2
    {
        friend struct trs2<fp>;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::scale2<fp>>;

      private:
        low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> vec;

      public:
        inline scale2(const point2<fp>& v): vec(v.vec()) {};
        inline scale2(): vec(sixit::units::create_dimensionless_scalar<fp>(1.0f), 
                              sixit::units::create_dimensionless_scalar<fp>(1.0f)) {};
        inline scale2(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> s): vec(s, s) {};
        inline scale2(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> x, 
                        sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> y): vec(x, y) {};

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"scale2">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "x", obj.vec.x);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "y", obj.vec.y);
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp>
    struct trs3
    {
        template <typename MeshBuffersT>
        friend class graphics::skinned_mesh3_rigging;
        friend class graphics::bone_formatter;
        friend struct sixit::lwa::fmt::formatter<sixit::geometry::trs3<fp>>;
        friend struct affine_transform3<fp>;
        template <typename fp1>
        friend gpu::mat4<fp1> low_level::for_renderers_only::matrix_dont_use_outside_of_renderer(const trs3<fp1>& trs);

    private:
#ifdef TRS_AS_MATRIX4X4
        low_level::matrix4x4<fp> mat;
#else
        translation3<fp> translation;
        rotation3<fp> rotation;
        scale3<fp> scale;
#endif
        template <sixit::units::physical_dimension dim>      
        inline low_level::dimensional_vector3<fp, dim> transform(const low_level::dimensional_vector3<fp, dim>& v) const;

    public:

        inline trs3(const translation3<fp>& t, const rotation3<fp>& r, const scale3<fp>& s);

        inline trs3();
        inline std::tuple<translation3<fp>, rotation3<fp>, scale3<fp>> trs() const;
        inline translation3<fp> transform(const translation3<fp>& v) const;
        inline rotation3<fp> transform(const rotation3<fp>& r) const;
        inline scale3<fp> transform(const scale3<fp>& s) const;
        inline point3<fp> transform(const point3<fp>& p) const;
        inline trs3 inverse() const;
        inline trs3 transform(const trs3& trs) const;
        inline point3<fp> multiply_point3x4(const point3<fp>& p) const;
        inline bool operator==(const trs3& other) const;

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser);
    };

    template <typename fp>
    struct trs2
    {
        friend struct  sixit::lwa::fmt::formatter<sixit::geometry::trs2<fp>>;

      private:
        translation2<fp> translation;
        rotation2<fp> rotation;
        scale2<fp> scale;

      public:
        inline trs2() : translation({}) {}
        inline trs2(const translation2<fp>& t, const rotation2<fp>& r, const scale2<fp>& s): translation{t}, rotation{r}, scale{s} {}

        inline std::tuple<translation2<fp>, rotation2<fp>, scale2<fp>> trs() const;

        inline translation2<fp> transform(const translation2<fp>& v) const;
        inline rotation2<fp> transform(const rotation2<fp>& r) const;
        inline scale2<fp> transform(const scale2<fp>& s) const;
        inline point2<fp> transform(const point2<fp>& p) const;

        inline trs2 inverse() const;

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"trs2">(comparser, obj);
            sixit::rw::read_write<&trs2::translation, "t2", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&trs2::rotation, "r2", sixit::rw::STRUCT>(comparser);
            sixit::rw::read_write<&trs2::scale, "s2", sixit::rw::STRUCT>(comparser);
            sixit::rw::end_struct(comparser);
        }
    };

}; // namespace geometry

namespace geometry 
{
  using rotation3f = rotation3<float>;
  using rotation2f = rotation2<float>;
  using rotation1f = rotation1<float>;
  using translation3f = translation3<float>;
  using translation2f = translation2<float>;
  using scale3f = scale3<float>;
  using scale2f = scale2<float>;
  using trs3f = trs3<float>;
  using trs2f = trs2<float>;
}

}; // namespace sixit

#endif //sixit_geometry_rotation_h_included

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
