/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_rotation_impl_h_included
#define sixit_geometry_rotation_impl_h_included

#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/quaternion.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/units.h"

namespace sixit
{
namespace geometry
{

    template <typename fp>
    rotation3<fp>::rotation3(const direction3<fp>& v1, const direction3<fp>& v2)
    {
        rotation3<fp>::q = low_level::dimensional_quaternion<fp, sixit::units::simple_scalar::dim>::from_to_rotation(v1.vec(), v2.vec());
    }

    template <typename fp>
    rotation3<fp>::rotation3(): q(low_level::quaternion<fp>::identity())
    {}

    template <typename fp>
    rotation3<fp>::rotation3(radians<fp> roll, radians<fp> pitch, radians<fp> yaw)
    {
        low_level::dimensional_quaternion q_roll = low_level::dimensional_quaternion<fp, sixit::units::simple_scalar::dim>::euler_angles(
                                                                                        sixit::units::create_dimensionless_scalar<fp>(degrees(roll).value_), 
                                                                                        sixit::units::create_dimensionless_scalar<fp>(0.0f), 
                                                                                        sixit::units::create_dimensionless_scalar<fp>(0.0f));
        
        low_level::dimensional_quaternion q_pitch = low_level::dimensional_quaternion<fp, sixit::units::simple_scalar::dim>::euler_angles(
                                                                                        sixit::units::create_dimensionless_scalar<fp>(0.0f), 
                                                                                        sixit::units::create_dimensionless_scalar<fp>(degrees(pitch).value_), 
                                                                                        sixit::units::create_dimensionless_scalar<fp>(0.0f));
        
        low_level::dimensional_quaternion q_yaw = low_level::dimensional_quaternion<fp, sixit::units::simple_scalar::dim>::euler_angles(
                                                                                        sixit::units::create_dimensionless_scalar<fp>(0.0f), 
                                                                                        sixit::units::create_dimensionless_scalar<fp>(0.0f), 
                                                                                        sixit::units::create_dimensionless_scalar<fp>(degrees(yaw).value_));

        q = q_roll * q_pitch * q_yaw;
    }
    
    template <typename fp>
	rotation3<fp>::rotation3(const direction3<fp>& axis, radians<fp> angle)
	{
		q = low_level::quaternion<fp>::axis_angle(axis.vec(),degrees(angle).value_);
	}
    
    template <typename fp>
    rotation3<fp>::rotation3(degrees<fp> roll, degrees<fp> pitch, degrees<fp> yaw)
        : rotation3(radians(roll), radians(pitch), radians(yaw))
    {}

    template <typename fp>
    rotation3<fp>::rotation3(axis_x_tag, radians<fp> angle)
    {
        q = low_level::dimensional_quaternion<fp, sixit::units::simple_scalar::dim>::axis_angle(low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>(
                    sixit::units::create_dimensionless_scalar<fp>(1.0f), 
                    sixit::units::create_dimensionless_scalar<fp>(0.0f), 
                    sixit::units::create_dimensionless_scalar<fp>(0.0f)), 
                    sixit::units::create_dimensionless_scalar<fp>(degrees(angle.value_).value_));
    }

    template <typename fp>
    rotation3<fp>::rotation3(axis_y_tag, radians<fp> angle)
    {
        q = low_level::quaternion<fp>::axis_angle(low_level::vector3(0.0f, 1.0f, 0.0f), degrees(angle.value_).value_);
    }

    template <typename fp>
    rotation3<fp>::rotation3(axis_z_tag, radians<fp> angle)
    {
        q = low_level::quaternion<fp>::axis_angle(low_level::vector3(0.0f, 0.0f, 1.0f), degrees(angle.value_).value_);
    }
    
    template <typename fp>
    rotation3<fp> rotation3<fp>::opposite() const
    {
        rotation3 r = *this;
        r.q = low_level::dimensional_quaternion<fp, sixit::units::simple_scalar::dim>::inverse(q);
        return r;
    }

    template <typename fp>
    void rotation3<fp>::flip()
    {
        q = low_level::quaternion<fp>::inverse(q);
    }

    template <typename fp>
    rotation3<fp> rotation3<fp>::operator+(const rotation3& o) const
    {
        rotation3 res = *this;
        res.q = (res.q * o.q).normalized();
        return res;
    }

    template <typename fp>
    rotation3<fp>& rotation3<fp>::operator+=(const rotation3& o)
    {
        q = (q * o.q).normalized();
        return *this;
    }

    template <typename fp>
    inline low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> rotation3<fp>::euler_angles() const
    {
        return q.euler_angles();
    }

    template <typename fp>
    direction3<fp> rotation3<fp>::rotate(const direction3<fp>& dir) const
    {
        low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> u(
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(q.x()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(q.y()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::simple_scalar>(q.z()));

        auto s = sixit::units::create_dimensionless_scalar<fp>(q.w());

        auto duv = low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::dot(u, dir.vec());
        auto duu = low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::dot(u, u);
        low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> cuv = low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim>::cross(u, dir.vec());
      //  vprime = 2.0f * dot(u, v) * u
     //       + (s * s - dot(u, u)) * v
     //       + 2.0f * s * cross(u, v);

        low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> res =
            u * (duv * sixit::units::create_dimensionless_scalar<fp>(2.f)) +
            dir.vec() * (s * s - duu) +
            cuv * (s * sixit::units::create_dimensionless_scalar<fp>(2.f));

        return direction3<fp>(res);
    }

    template<typename fp>
    direction2<fp> rotation2<fp>::rotate(const direction2<fp>& dir) const
    {
       /* low_level::vector3 u(q.x, q.y, q.z);

        fp s = q.w;
        low_level::vector3 dir3(dir.vec().x, dir.vec().y, 0.0f);
        fp duv = low_level::vector3::dot(u, dir3);
        fp duu = low_level::vector3::dot(u, u);
        low_level::vector3 cuv = low_level::vector3::cross(u, dir3);
        low_level::vector3 res =
            u * (2.0f * duv) +
            dir3 * (s * s - duu) +
            cuv * (2.0f * s);*/

        low_level::complex_number_rotation d(dir.vec().x, dir.vec().y);
        d *= q;

        return direction2<fp>(low_level::dimensional_vector2(d.a, d.b));
    }

    // rotation2
    template<typename fp>
    rotation2<fp>::rotation2(const direction2<fp>& d1, const direction2<fp>& d2)
    {
        low_level::dimensional_vector2 v1 = d1.vec();
        low_level::dimensional_vector2 v2 = d2.vec();
       // q = low_level::quaternion::from_to_rotation(low_level::vector3(v1.x, v1.y, 0.0f), low_level::vector3(v2.x, v2.y, 0.0f));
        q = low_level::complex_number_rotation<fp>::from_to_rotation(v1, v2);
    }

    template<typename fp>
    rotation2<fp>::rotation2(): q{low_level::complex_number_rotation<fp>::identity()}
    {}

    template<typename fp>
    rotation2<fp> rotation2<fp>::opposite() const
    {
        rotation2<fp> r = *this;
        r.q = low_level::complex_number_rotation<fp>::inverse(q);
        return r;
    }

    template<typename fp>
    void rotation2<fp>::flip()
    {
        q = low_level::complex_number_rotation<fp>::inverse(q);
    }

    template<typename fp>
    rotation2<fp> rotation2<fp>::operator+(const rotation2<fp>& o) const
    {
        rotation2<fp> res = *this;
        res.q *= o.q;
        return res;
    }

    template<typename fp>
    rotation2<fp>& rotation2<fp>::operator+=(const rotation2<fp>& o)
    {
        q *= o.q;
        return *this;
    }

    template<typename fp>
    rotation2<fp> rotation2<fp>::right_angle()
    {
        rotation2<fp> res;
        res.q = low_level::complex_number_rotation<fp>::by_angle(sixit::units::create_dimensionless_scalar<fp>(fp(90.0f)));
        return res;
    }

    template<typename fp>
    rotation2<fp> rotation2<fp>::straight_angle()
    {
        rotation2<fp> res;
        res.q = low_level::complex_number_rotation<sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>>::by_angle(sixit::units::create_dimensionless_scalar<fp>(fp(180.0f)));
        return res;
    }

    template<typename fp>
    rotation2<fp> rotation2<fp>::orthogonal() const
    {
        return *this + rotation2<fp>::right_angle();
    }

    template<typename fp>
    sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> rotation2<fp>::euler_angle() const
    {
        return q.angle();
    }

    template<typename fp>
    rotation1<fp>::rotation1(const direction1<fp>& v1, const direction1<fp>& v2)
    {
        rotation = v1.vec() > 0.0f && v2.vec() > 0.0f; //TODO: check this
    }

    template<typename fp>
    rotation1<fp> rotation1<fp>::opposite() const
    {
        return rotation1(!rotation);
    }

    template<typename fp>
    void rotation1<fp>::flip()
    {
        rotation = !rotation;
    }

    template<typename fp>
    rotation1<fp> rotation1<fp>::operator+(const rotation1<fp>&) const
    {
        rotation1 r = *this;
        r.rotation = !r.rotation;
        return r;
    }

    template<typename fp>
    rotation1<fp>& rotation1<fp>::operator+=(const rotation1<fp>&)
    {
        rotation = !rotation;
        return *this;
    }

    template<typename fp>
    rotation1<fp> rotation1<fp>::right_angle()
    {
        return rotation1(true);
    }

    template<typename fp>
    rotation1<fp> rotation1<fp>::left_angle()
    {
        return rotation1(false);
    }


#ifdef TRS_AS_MATRIX4X4

    template <typename fp>
    inline trs3<fp>::trs3(const translation3& t, const rotation3& r, const scale3& s) : mat(low_level::matrix4x4::trs(t.vec, r.q, s.vec)) { }

    template <typename fp>
    inline trs3<fp>::trs3() : mat(low_level::matrix4x4::identity()) { }

    template <typename fp>
    std::tuple<translation3, rotation3, scale3> trs3<fp>::trs() const
    {
        translation3 t(point3{});
        rotation3 r;
        scale3 s;
        low_level::matrix4x4::decompose(mat,t.vec,r.q,s.vec);
        return { t,r,s };
    }

    template <typename fp>
    low_level::vector3 trs3<fp>::transform(const low_level::vector3& v) const
    {
        //this is correct
        auto res = multiply_point3x4(v);    //this gives different result as TRS_AS_MATRIX4X4 implementation
        return res.vec();

        //auto res = mat.multiply_point3x4(v);    //incorrect result. matrix4x4 has switched colums x rows compared to trs
        //return res;
    }

    template <typename fp>
    translation3 trs3<fp>::transform(const translation3& v) const
    {
        low_level::vector3 t = mat[3];
        return translation3(t + v.vec);
    }

    template <typename fp>
    rotation3 trs3<fp>::transform(const rotation3& v) const
    {
        translation3 t(point3{});
        rotation3 r;
        scale3 s;
        low_level::matrix4x4::decompose(mat, t.vec, r.q, s.vec);
        return v + r;
    }

    template <typename fp>
    scale3 trs3<fp>::transform(const scale3& v) const
    {
        low_level::vector3 s{ mat[0].magnitude(), mat[1].magnitude(), mat[2].magnitude() };

        if (gpu::mat4::determinant(mat) < 0)
            s *= -1.0f;

        return scale3(s * v.vec);
    }

    template <typename fp>
    point3 trs3<fp>::transform(const point3& p) const
    {
        return transform(p.vec());
    }

    template <typename fp>
    trs3 trs3<fp>::transform(const trs3& o) const
    {
        trs3 trs;
        trs.mat = mat *o.mat;
        return trs;
    }

    template <typename fp>
    point3<fp> trs3<fp>::multiply_point3x4(const point3<fp>& p) const
    {
        // not the same as low_level::matrix4x4::multiply_point3x4(p.vec());
        // keep this method to maintain same result as trs without TRS_AS_MATRIX4X4
        const fp y = mat[0][1] * p.vec().x + mat[1][1] * p.vec().y + mat[2][1] * p.vec().z + mat[3][1];
        const fp x = mat[0][0] * p.vec().x + mat[1][0] * p.vec().y + mat[2][0] * p.vec().z + mat[3][0];
        const fp z = mat[0][2] * p.vec().x + mat[1][2] * p.vec().y + mat[2][2] * p.vec().z + mat[3][2];
        return low_level::vector3<fp>(x, y, z);
    }

    template <typename fp>
    inline bool trs3<fp>::operator==(const trs3& other) const
    {
        return (mat[0] == other.mat[0] && mat[1] == other.mat[1] && mat[2] == other.mat[2] && mat[3] == other.mat[3]);
    }
#else

    template <typename fp>
    inline trs3<fp>::trs3(const translation3<fp>& t, const rotation3<fp>& r, const scale3<fp>& s) : translation(t), rotation(r), scale(s) { }

    template <typename fp>
    inline trs3<fp>::trs3() : translation(point3<fp>()){ }

    template <typename fp>
    std::tuple<translation3<fp>, rotation3<fp>, scale3<fp>> trs3<fp>::trs() const
    {
        return std::make_tuple(translation, rotation, scale);
    }

    template <typename fp>
    template <sixit::units::physical_dimension dim>
    low_level::dimensional_vector3<fp, dim> trs3<fp>::transform(const low_level::dimensional_vector3<fp, dim>& v) const
    {
        low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> r = v;
        r *= scale.vec;
        r *= rotation.q;
        r += translation.vec;
        return r;
    }

    template <typename fp>
    rotation3<fp> trs3<fp>::transform(const rotation3<fp>& v) const
    {
        rotation3 r = v;
        r += rotation;
        return r;
    }

    template <typename fp>
    translation3<fp> trs3<fp>::transform(const translation3<fp>& v) const
    {
        return translation3<fp>(point3<fp>(v.vec + translation.vec));
    }

    template <typename fp>
    scale3<fp> trs3<fp>::transform(const scale3<fp>& v) const
    {
        low_level::vector3 r = v.vec;
        r *= scale.vec;
        return scale3(point3(
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(r.x()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(r.y()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(r.z())));
    }

    template <typename fp>
    point3<fp> trs3<fp>::transform(const point3<fp>& v) const
    {
        low_level::dimensional_vector3<fp, sixit::units::length_unit::dim> vv(
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(v.vec().x()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(v.vec().y()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(v.vec().z()), 
            typename low_level::dimensional_vector3<fp, sixit::units::length_unit::dim>::position_type_t());

        low_level::dimensional_vector3<fp, sixit::units::length_unit::dim> r = transform(vv);
        return point3<fp>(
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(r.x()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(r.y()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(r.z()));
    }

    template <typename fp>
    trs3<fp> trs3<fp>::inverse() const
    {
        // Inverse Translation
        point3<fp> inv_translation_point(
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(-translation.vec.x()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(-translation.vec.y()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(-translation.vec.z()));
        translation3<fp> inv_translation = translation3<fp>(inv_translation_point);

        // Inverse Rotation
        rotation3<fp> inv_rotation = rotation.opposite();

        // Inverse Scale
        point3<fp> inv_scale_point(
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.0f / scale.vec.x()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.0f / scale.vec.y()), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(1.0f / scale.vec.z()));
        scale3<fp> inv_scale = scale3<fp>(inv_scale_point);

        return trs3<fp>(inv_translation, inv_rotation, inv_scale);
    }

    template <typename fp>
    trs3<fp> trs3<fp>::transform(const trs3<fp>& o) const
    {
        // translation = rotated and scaled by parent, child translation
        low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> t = o.translation.vec * scale.vec * rotation.q + translation.vec;
        // scale = parent scale * child scale
        low_level::dimensional_vector3<fp, sixit::units::simple_scalar::dim> s = scale.vec * o.scale.vec;
        // rotation = parent rotation * child rotation
        rotation3<fp> r = rotation + o.rotation;
        return trs3<fp>(translation3<fp>(point3<fp>(t)), r, scale3<fp>(point3<fp>(s)));
    }

    template <typename fp>
    point3<fp> trs3<fp>::multiply_point3x4(const point3<fp>& p) const
    {
        low_level::matrix4x4<fp> m = low_level::matrix4x4<fp>::trs(translation.vec, rotation.q, scale.vec);
        const fp x = m.mat[0][0] * p.vec().x() + m.mat[1][0] * p.vec().y() + m.mat[2][0] * p.vec().z() + m.mat[3][0];
        const fp y = m.mat[0][1] * p.vec().x() + m.mat[1][1] * p.vec().y() + m.mat[2][1] * p.vec().z() + m.mat[3][1];
        const fp z = m.mat[0][2] * p.vec().x() + m.mat[1][2] * p.vec().y() + m.mat[2][2] * p.vec().z() + m.mat[3][2];
        point3<fp> pr(
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(x), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(y), 
            sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(z));
        return pr;
    }

    template <typename fp>
    bool trs3<fp>::operator==(const trs3& other) const
    {
        return (translation == other.translation) && (rotation == other.rotation) && (scale == other.scale);
    }

#endif // TRS_AS_MATRIX4X4

    
    template <typename fp>
    template <typename This, typename ComparserT>
    inline void trs3<fp>::read_write(This& obj, ComparserT& comparser)
    {
#ifdef TRS_AS_MATRIX4X4
        _internal::trs3converter converter;
        sixit::rw::read_write<&trs3<fp>::mat, "mat", sixit::rw::STRUCT>(comparser, converter);
#else
        
        sixit::rw::begin_struct<"trs3">(comparser, obj);
        sixit::rw::read_write<&trs3<fp>::translation, "t3", sixit::rw::STRUCT>(comparser);
        sixit::rw::read_write<&trs3<fp>::rotation, "r3", sixit::rw::STRUCT>(comparser);
        sixit::rw::read_write<&trs3<fp>::scale, "s3", sixit::rw::STRUCT>(comparser);
        sixit::rw::end_struct(comparser);
#endif
    }

    template<typename fp>
    std::tuple<translation2<fp>, rotation2<fp>, scale2<fp>> trs2<fp>::trs() const
    {
        return std::make_tuple(translation, rotation, scale);
    }

    template<typename fp>
    translation2<fp> trs2<fp>::transform(const translation2<fp>& v) const
    {
        return translation2<fp>(v.vec + translation.vec);
    }

    template<typename fp>
    rotation2<fp> trs2<fp>::transform(const rotation2<fp>& r) const
    {
        rotation2 new_rot = r;
        new_rot += rotation;
        return new_rot;
    }

    template<typename fp>
    scale2<fp> trs2<fp>::transform(const scale2<fp>& s) const
    {
        low_level::vector2 new_scale = s.vec;
        new_scale *= scale.vec;
        return scale2<fp>(new_scale.x, new_scale.y);
    }

    template<typename fp>
    point2<fp> trs2<fp>::transform(const point2<fp>& p) const
    {
        // Scaling
        low_level::vector2<fp> scaled = p.vec() * scale.vec; 

        // Rotation
        low_level::complex_number_rotation<fp> scaled_complex(scaled.x, scaled.y);
        low_level::complex_number_rotation<fp> rotated_complex = rotation.q * scaled_complex;
        low_level::vector2<fp> rotated(rotated_complex.a, rotated_complex.b);

        // Translation
        low_level::vector2<fp> translated = rotated + translation.vec;

        return point2<fp>(translated.x, translated.y);
    }

    template<typename fp>
    trs2<fp> trs2<fp>::inverse() const
    {
        // Inverse Translation
        point2<fp> inv_translation_point(-translation.vec.x, -translation.vec.y);
        translation2<fp> inv_translation = translation2(inv_translation_point);

        // Inverse Rotation
        rotation2<fp> inv_rotation = rotation.opposite();

        // Inverse Scale
        point2<fp> inv_scale_point(1.0f / scale.vec.x, 1.0f / scale.vec.y);
        scale2<fp> inv_scale = scale2(inv_scale_point);

        return trs2<fp>(inv_translation, inv_rotation, inv_scale);
    }
};
};

#endif //sixit_geometry_rotation_impl_h_included

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
