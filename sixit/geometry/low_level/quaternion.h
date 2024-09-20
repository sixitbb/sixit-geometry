/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_quaternion_h_included
#define sixit_geometry_low_level_quaternion_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/matrix.h"
#include "sixit/geometry/_internal/simd/quat4.h"

namespace sixit
{
namespace geometry
{
namespace low_level
{

  template <typename fp>
  struct quaternion: public gpu::quat4<fp>
  {
      friend struct sixit::lwa::fmt::formatter<::sixit::geometry::rotation3<fp>>;
      friend struct sixit::lwa::fmt::formatter<::sixit::geometry::rotation2<fp>>;
      friend struct sixit::lwa::fmt::formatter<quaternion>;
      friend struct matrix4x4<fp>;
      friend struct matrix3x3<fp>;
      friend struct vector3<fp>;
      template <typename fp1, sixit::units::physical_dimension dim1>
      friend struct dimensional_vector3;
      template <typename fp1>
      friend struct ::sixit::geometry::rotation3;
      template <typename fp1>
      friend struct ::sixit::geometry::rotation2;
      friend struct ::sixit::geometry::trs3<fp>;
      friend struct ::sixit::graphics::trs3_animation;
      friend class ::sixit::graphics::skeleton_joint;

      template <typename fp1> friend ::sixit::geometry::rotation3<fp1> for_renderers_only::make_rotation3_dont_use_outside_of_renderer(fp1 x, fp1 y, fp1 z, fp1 w);
      template <typename fp1> friend ::sixit::geometry::rotation3<fp1> for_importer_only::make_rotation3_dont_use_outside_of_importer(fp1 x, fp1 y, fp1 z, fp1 w);
      template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> for_renderers_only::xyzw_dont_use_outside_of_renderer(const ::sixit::geometry::rotation3<fp1>& r);
      template <typename fp1> friend gpu::mat4<fp1> for_renderers_only::matrix_dont_use_outside_of_renderer(const ::sixit::geometry::trs3<fp1>& trs);
      template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> for_importer_only::xyzw_dont_use_outside_of_importer(const geometry::rotation3<fp1>& rt3);
      template <typename fp1> friend rotation3<fp1> for_data_transfers_only::make_rotation3_dont_use_outside_of_data_transfers(fp1 x, fp1 y, fp1 z, fp1 w);
      template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> for_data_transfers_only::xyzw_dont_use_outside_of_data_transfers(const rotation3<fp1>& r);
  private:
      explicit inline quaternion(const gpu::quat4<fp>& o);
      inline quaternion(const matrix3x3<fp>& m);
      inline quaternion();
      inline quaternion(const gpu::mat4<fp>& m);
      inline quaternion(fp w, fp x, fp y, fp z);
      inline quaternion(fp x, fp y, fp z);

      template <typename This, typename ComparserT>
      static void read_write(This& obj, ComparserT& comparser)
      {
          sixit::rw::begin_struct<"quaternion">(comparser, obj);
          sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "x", obj.x());
          sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "y", obj.y());
          sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "z", obj.z());
          sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "w", obj.w());
          sixit::rw::end_struct(comparser);
      }


      inline explicit operator gpu::mat4<fp>() const;
      inline static quaternion create_from_matrix(const gpu::mat4<fp>& mat);
      inline static quaternion create_from_matrix(const matrix3x3<fp>& mat);
      inline static quaternion axis_angle(const vector3<fp>& axis, fp angle);
      inline static quaternion euler_angles(fp x, fp y, fp z);
      inline static quaternion identity();
      inline vector3<fp> euler_angles() const;
      inline quaternion normalized() const;
      inline void set(fp w, fp x, fp y, fp z);
      inline void normalize();
      inline static fp angle(const quaternion& a, const quaternion& b);
      inline static fp dot(const quaternion& a, const quaternion& b);
      inline static quaternion inverse(const quaternion& a);
      // TODO: test this
      inline static quaternion look_rotation(const vector3<fp>& fwd_dir, const vector3<fp>& up_dir);
      inline static quaternion from_to_rotation(const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& from_direction, 
                                                  const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& to_direction);
      inline quaternion operator*(const quaternion& o);
      inline quaternion& operator*=(const quaternion& o);
      inline quaternion operator+(const quaternion& o);
      inline quaternion& operator+=(const quaternion& o);
      inline quaternion operator-(const quaternion& o);
      inline quaternion& operator-=(const quaternion& o);
      inline quaternion operator*(fp o);
      inline quaternion& operator*=(fp o);
      inline quaternion operator/(fp o);
      inline quaternion& operator/=(fp o);

      [[nodiscard]] inline bool _for_test_only_approximate_eq(const quaternion& other, size_t n = 1) const;
  };


  template <typename fp, sixit::units::physical_dimension dim>
  struct dimensional_quaternion: public gpu::quat4<fp>
  {
      template <typename fp1, sixit::units::physical_dimension dim1>
      friend struct dimensional_quaternion;
      friend struct sixit::lwa::fmt::formatter<::sixit::geometry::rotation3<fp>>;
      friend struct sixit::lwa::fmt::formatter<::sixit::geometry::rotation2<fp>>;
      friend struct sixit::lwa::fmt::formatter<dimensional_quaternion>;
      friend struct matrix4x4<fp>;
      friend struct matrix3x3<fp>;
      friend struct vector3<fp>;
      template <typename fp1, sixit::units::physical_dimension dim1>
      friend struct dimensional_vector3;
      template <typename fp1>
      friend struct ::sixit::geometry::rotation3;
      template <typename fp1>
      friend struct ::sixit::geometry::rotation2;
      friend struct ::sixit::geometry::trs3<fp>;
      friend struct ::sixit::graphics::trs3_animation;
      friend class ::sixit::graphics::skeleton_joint;

      template <typename fp1> friend ::sixit::geometry::rotation3<fp1> for_renderers_only::make_rotation3_dont_use_outside_of_renderer(fp1 x, fp1 y, fp1 z, fp1 w);
      template <typename fp1> friend ::sixit::geometry::rotation3<fp1> for_importer_only::make_rotation3_dont_use_outside_of_importer(fp1 x, fp1 y, fp1 z, fp1 w);
      template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> for_renderers_only::xyzw_dont_use_outside_of_renderer(const ::sixit::geometry::rotation3<fp1>& r);
      template <typename fp1> friend gpu::mat4<fp1> for_renderers_only::matrix_dont_use_outside_of_renderer(const ::sixit::geometry::trs3<fp1>& trs);
      template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> for_importer_only::xyzw_dont_use_outside_of_importer(const geometry::rotation3<fp1>& rt3);
      template <typename fp1> friend rotation3<fp1> for_data_transfers_only::make_rotation3_dont_use_outside_of_data_transfers(fp1 x, fp1 y, fp1 z, fp1 w);
      template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> for_data_transfers_only::xyzw_dont_use_outside_of_data_transfers(const rotation3<fp1>& r);

      template <typename This, typename ComparserT>
      static void read_write(This& obj, ComparserT& comparser)
      {
          sixit::rw::begin_struct<"quaternion">(comparser, obj);
          sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "x", obj.x());
          sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "y", obj.y());
          sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "z", obj.z());
          sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "w", obj.w());
          sixit::rw::end_struct(comparser);
      }

    private:
      explicit inline dimensional_quaternion(const gpu::quat4<fp>& o);
      inline dimensional_quaternion(const matrix3x3<fp>& m);
      inline dimensional_quaternion();
      inline dimensional_quaternion(const gpu::mat4<fp>& m);
      inline dimensional_quaternion(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> w, 
                                    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> x, 
                                    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> y, 
                                    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> z);
      inline dimensional_quaternion(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> x, 
                                    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> y, 
                                    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> z);


      inline sixit::units::dimensional_scalar<fp, dim> dim_x() const { return sixit::units::dimensional_scalar<fp, dim>({ this->x(), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() }); }
      inline sixit::units::dimensional_scalar<fp, dim> dim_y() const { return sixit::units::dimensional_scalar<fp, dim>({ this->y(), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() }); }
      inline sixit::units::dimensional_scalar<fp, dim> dim_z() const { return sixit::units::dimensional_scalar<fp, dim>({ this->z(), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() }); }
      inline sixit::units::dimensional_scalar<fp, dim> dim_w() const { return sixit::units::dimensional_scalar<fp, dim>({ this->w(), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() }); }

      inline explicit operator gpu::mat4<fp>() const;
      inline static dimensional_quaternion create_from_matrix(const gpu::mat4<fp>& mat);
      inline static dimensional_quaternion create_from_matrix(const matrix3x3<fp>& mat);
      inline static dimensional_quaternion axis_angle(const dimensional_vector3<fp, dim>& axis, 
                                    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle);
      inline static dimensional_quaternion euler_angles(
                                    sixit::units::dimensional_scalar<fp, dim> x, 
                                    sixit::units::dimensional_scalar<fp, dim> y, 
                                    sixit::units::dimensional_scalar<fp, dim> z);
      inline static dimensional_quaternion identity();
      inline dimensional_vector3<fp, dim> euler_angles() const;
      inline dimensional_quaternion<fp, sixit::units::simple_scalar::dim> normalized() const;

      inline void set(sixit::units::dimensional_scalar<fp, dim> w, sixit::units::dimensional_scalar<fp, dim> x, 
                        sixit::units::dimensional_scalar<fp, dim> y, sixit::units::dimensional_scalar<fp, dim> z);
      inline static sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle(
                                                            const dimensional_quaternion<fp, dim>& a, 
                                                            const dimensional_quaternion<fp, dim>& b);
      inline static sixit::units::dimensional_scalar<fp, dim * dim> dot(const dimensional_quaternion<fp, dim>& a, const dimensional_quaternion<fp, dim>& b);
      inline static dimensional_quaternion inverse(const dimensional_quaternion& a);
      // TODO: test this
      inline static dimensional_quaternion look_rotation(const dimensional_vector3<fp, dim>& fwd_dir, 
                                                          const dimensional_vector3<fp, dim>& up_dir);
      inline static dimensional_quaternion from_to_rotation(const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& from_direction, 
                                                  const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& to_direction);
      template <sixit::units::physical_dimension dim1>
      inline dimensional_quaternion<fp, dim * dim1> operator*(const dimensional_quaternion<fp, dim1>& o);
      inline dimensional_quaternion<fp, dim>& operator*=(const dimensional_quaternion<fp, sixit::units::simple_scalar::dim>& o);
      inline dimensional_quaternion<fp, dim> operator+(const dimensional_quaternion<fp, dim>& o);
      inline dimensional_quaternion<fp, dim>& operator+=(const dimensional_quaternion<fp, dim>& o);
      inline dimensional_quaternion<fp, dim> operator-(const dimensional_quaternion<fp, dim>& o);
      inline dimensional_quaternion<fp, dim>& operator-=(const dimensional_quaternion<fp, dim>& o);
      template <sixit::units::physical_dimension dim1>
      inline dimensional_quaternion<fp, dim * dim1> operator*(sixit::units::dimensional_scalar<fp, dim1> o);
      inline dimensional_quaternion<fp, dim>& operator*=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);
      template <sixit::units::physical_dimension dim1>
      inline dimensional_quaternion<fp, dim / dim1> operator/(sixit::units::dimensional_scalar<fp, dim1> o);
      inline dimensional_quaternion<fp, dim>& operator/=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);

      [[nodiscard]] inline bool _for_test_only_approximate_eq(const dimensional_quaternion<fp, dim>& other, size_t n = 1) const;
  };

}; // namespace low_level

namespace low_level 
{
    // aliases 
    using quaternionf = quaternion<float>;

    template <sixit::units::physical_dimension dim>
    using dimensional_quaternionf = dimensional_quaternion<float, dim>;
}

}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_level_quaternion_h_included

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
