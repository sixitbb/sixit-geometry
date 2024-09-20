/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_vector_h_included
#define sixit_geometry_low_level_vector_h_included

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/_internal/simd/vec4p.h"
#include "sixit/dmath/fp_approximate_eq.h"
#include "sixit/rw/comparsers/_internal/comparsers_common.h"
#include "sixit/core/units.h"

#include "sixit/core/lwa.h"


namespace sixit
{
namespace graphics
{
    class intersectable_mesh;
}
namespace geometry
{
    enum class XYZ;
    template <typename fp, XYZ axis>
    struct axis_oriented_ray2;
    class voxelizer;

    template<typename PCloud> plane3<float> best_fit_plane3(const PCloud& vlo);
    template<typename VLO> plane3<float> best_fit_plane3_vlo(const VLO& vlo);
    
    // template<typename fp, typename TSetPixel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
    // void supercover_line(const line_segment2<fp> &ls2, TSetPixel& SET_PIXEL);

    // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
    // void triangle_voxelize(const triangle3& trng, TSetVoxel& SET_VOXEL);

    // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
    // void line_voxelize(const line_segment3& ls3, TSetVoxel& SET_VOXEL);

    namespace for_renderers_only {
        template <typename fp>
        struct frustum;
    }

namespace low_level
{
    namespace impl
    {
        template <typename fp, typename fp2 = fp, std::size_t N>
        [[nodiscard]] inline bool _for_test_only_approximate_eq(const fp* left, const fp2* right, size_t n = 1) 
        {
            // Suboptimal solution that works for now (floating-point is hard)
            // This code may eventually fail in special cases: infinite, large values, or comparing 
            // a difference of large values agains zero (when least significant bit > epsilon)
            // Also, this epsilon may be too large for really small values
            for (std::size_t i = 0; i < N; ++i)
            {
                if (!sixit::dmath::test_helpers::approximate_eq(left[i], right[i], n))
                    return false;
            }
            return true;
        }
    }

    template <typename fp>
    struct vector2
    {
        template <typename fp1>
        friend struct vector2;
        friend struct sixit::lwa::fmt::formatter<point2<fp>>;
        friend struct sixit::lwa::fmt::formatter<vector2>;
        friend struct sixit::lwa::fmt::formatter<::sixit::geometry::translation2<fp>>;
        friend struct sixit::lwa::fmt::formatter<::sixit::geometry::scale2<fp>>;
        friend struct quaternion<fp>;
        friend struct matrix4x4<fp>;
        friend struct matrix3x3<fp>;
        
        friend struct vector3<fp>;
        
        friend struct ::sixit::geometry::shape2_base<fp>;
        friend struct ::sixit::geometry::indexed_point2<fp>;

        template <typename fp1> friend struct ::sixit::geometry::cbezier2;
        template <typename fp1> friend struct ::sixit::geometry::curve2;
        template <typename fp1> friend struct ::sixit::geometry::curved_polygon2;
        template <typename fp1> friend class ::sixit::geometry::curv_builder;
        
        friend class ::sixit::geometry::voxelizer;
        friend struct ::sixit::geometry::projection2to1<fp>;
        friend struct ::sixit::geometry::projection2to1ex<fp>;
        friend struct ::sixit::geometry::projection3to2noscale<fp>;
        friend struct ::sixit::geometry::projection_perspective2<fp>;
        friend struct ::sixit::geometry::trs2<fp>;
        friend struct ::sixit::geometry::translation2<fp>;
        friend struct ::sixit::geometry::scale2<fp>;
        
        friend struct ::sixit::geometry::affine_transform2to1<fp>;

        template<typename T> friend class ::sixit::graphics::full_mesh3;
        friend struct ::sixit::geometry::affine_transform2to1<fp>;
        friend struct ::sixit::geometry::affine_transform2<fp>;
       
        template<class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T)
        friend class ::sixit::graphics::mesh3_importer;

        // template<typename fp1, typename TSetPixel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xy, TSetPixel)
        // friend void (::sixit::geometry::supercover_line)(const ::sixit::geometry::line_segment2<fp1>& ls2, TSetPixel& SET_PIXEL);

      private:
        inline vector2(const vector2<fp>& o);
        inline vector2();
        inline vector2(fp x, fp y);
        inline vector2(const vector3<fp>& o);
        fp x;
        fp y;
        inline void set(fp x, fp y);
        inline static fp angle(const vector2<fp>& a, const vector2<fp>& b);
        inline static fp dot(const vector2<fp>& a, const vector2<fp>& b);
        static vector2<fp> max(const vector2<fp>& a, const vector2<fp>& b);
        static vector2<fp> min(const vector2<fp>& a, const vector2<fp>& b);
        inline fp magnitude() const;
        inline auto sqr_magnitude() const;
        inline void normalize();
        [[nodiscard]] fp normalized() const;
        inline void scale(fp b);
        inline static vector2<fp> scale(const vector2<fp>& a, fp b);
        /*inline static vector2<fp> up();
        inline static vector2<fp> down();
        inline static vector2<fp> left();
        inline static vector2<fp> right();
        inline static vector2<fp> one();*/
        inline static vector2<fp> zero();

      public:
        inline bool operator==(const vector2& o) const
        {
            return x == o.x && y == o.y;
        }
        template<typename fp2>
        inline bool operator==(const vector2<fp2>& o) const
        {
            return x == fp(o.x) && y == fp(o.y);
        }

        inline operator vector3<fp>() const;
        inline vector2<fp>& operator+=(const vector2<fp>& o);
        inline const vector2<fp> operator+(const vector2<fp>& o) const;
        inline vector2<fp>& operator+=(fp o);
        inline const vector2<fp> operator+(fp o) const;
        inline vector2<fp>& operator-=(const vector2<fp>& o);
        inline const vector2<fp> operator-(const vector2<fp>& o) const;
        inline vector2<fp>& operator-=(fp o);
        inline const vector2<fp> operator-(fp o) const;
        inline vector2<fp>& operator*=(const vector2<fp>& o);
        inline const vector2<fp> operator*(const vector2<fp>& o) const;
        inline vector2<fp>& operator*=(fp o);
        inline const vector2<fp> operator*(const fp o) const;
        inline vector2<fp>& operator/=(const vector2<fp>& o);
        inline const vector2<fp> operator/(const vector2<fp>& o) const;
        inline vector2<fp>& operator/=(fp o);
        inline const vector2<fp> operator/(fp o) const;
        inline vector2<fp>& operator*=(const matrix4x4<fp>& o);
        inline const vector2<fp> operator*(const matrix4x4<fp>& o) const;
        inline vector2<fp>& operator*=(const matrix3x3<fp>& o);
        inline const vector2<fp> operator*(const matrix3x3<fp>& o) const;
        inline vector2<fp>& operator*=(const quaternion<fp>& o);
        inline const vector2<fp> operator*(const quaternion<fp>& o) const;
        inline vector2<fp> operator-() const;
        inline fp operator[](int index) const;

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"vector2">(comparser, obj);
            sixit::rw::read_write<&vector2::x, "x", sixit::rw::F32>(comparser);
            sixit::rw::read_write<&vector2::y, "y", sixit::rw::F32>(comparser);
            sixit::rw::end_struct(comparser);
        }

        template<typename fp2>
        [[nodiscard]] inline bool _for_test_only_approximate_eq(const vector2<fp2>& other, size_t n = 1) const
        {
            return impl::_for_test_only_approximate_eq<fp, fp2, 1>(&x, &other.x, n)
                && impl::_for_test_only_approximate_eq<fp, fp2, 1>(&y, &other.y, n);
        }
    };

    template <typename fp, sixit::units::physical_dimension dim>
    struct dimensional_vector2
    {
        using element_t = sixit::units::dimensional_scalar<fp, dim>;

        template <typename fp1, sixit::units::physical_dimension dim1>
        friend struct dimensional_vector2;
        friend struct sixit::lwa::fmt::formatter<point2<fp>>;
        friend struct ::sixit::geometry::point2<fp>;
        friend struct ::sixit::geometry::direction2<fp>;
        friend struct ::sixit::geometry::barycentric<fp>;
        friend struct polygon2_impl<fp>;
        friend struct ::sixit::geometry::ray2<fp>;
        friend struct ::sixit::geometry::rotation2<fp>;
        friend struct ::sixit::geometry::line2<fp>;
        friend struct ls2_impl<fp>;
        friend struct tri2_impl<fp>;

        friend struct matrix3x3<fp>;

        friend struct ::sixit::geometry::projection2<fp>;
        friend struct ::sixit::geometry::projection3to2<fp>;
        friend struct ::sixit::geometry::projection2to1<fp>;
        friend struct ::sixit::geometry::projection2to1ex<fp>;
        friend struct ::sixit::geometry::affine_transform2to1<fp>;
        friend struct ::sixit::geometry::affine_transform2<fp>;
        friend struct ::sixit::geometry::mirror_transform2<fp>;
        friend struct ::sixit::geometry::projection2to1noscale<fp>;
        friend struct ::sixit::geometry::projection_perspective2<fp>;
        friend struct ::sixit::geometry::inverse_projection_perspective3<fp>;
        
        template <typename fp1> friend struct ::sixit::geometry::line_segment2;
        template <typename fp1> friend struct ::sixit::geometry::arc2;
        template <typename fp1> friend struct complex_number_rotation;
        template <typename fp1> friend struct ::sixit::geometry::qbezier2;
        template <typename fp1> friend struct ::sixit::geometry::hermit2;
        template <typename fp1> friend struct ::sixit::geometry::curve2;
        template <typename fp1> friend struct ::sixit::geometry::bounds2;
        template <typename fp1> friend class ::sixit::geometry::curv_builder;

        template <class Triangle, typename fp1> friend class ::sixit::geometry::triangle2_stack;
        template <typename fp1, ::sixit::geometry::XYZ axis> friend struct ::sixit::geometry::axis_oriented_ray2;

        friend class ::sixit::geometry::grid2<fp>;
        friend class ::sixit::graphics::texcoord_buffer_vec2;
        friend class ::sixit::graphics::texcoord_buffer_vec2_uint16;
        

        friend struct ::sixit::geometry::indexed_polygon2<fp>;
        friend struct ::sixit::geometry::polygon2<fp>;
        friend struct ::sixit::geometry::shape2<fp>;

        template <typename fp1>
        friend std::tuple<fp1, fp1> low_level::for_renderers_only::xy_dont_use_outside_of_renderer(const ::sixit::geometry::point2<fp1>& p);
        template <typename fp1> 
        friend std::tuple<fp1, fp1, fp1, fp1> low_level::for_renderers_only::xy_dont_use_outside_of_renderer(const line_segment2<fp1>& p);

        template<class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T)
        friend class ::sixit::graphics::mesh3_importer;
    
      private:
        inline dimensional_vector2(const dimensional_vector2<fp, dim>& o);
        inline dimensional_vector2();
        inline dimensional_vector2(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y);
        inline dimensional_vector2(const dimensional_vector3<fp, dim>& o);
        sixit::units::dimensional_scalar<fp, dim> x;
        sixit::units::dimensional_scalar<fp, dim> y;
        inline void set(sixit::units::dimensional_scalar<fp, dim> x_, sixit::units::dimensional_scalar<fp, dim> y_);
        inline static sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle(const dimensional_vector2<fp, dim>& a, const dimensional_vector2<fp, dim>& b);
        template <sixit::units::physical_dimension dim_o> 
        inline static sixit::units::dimensional_scalar<fp, dim * dim_o> dot(const dimensional_vector2<fp, dim>& a, const dimensional_vector2<fp, dim_o>& b);
        static dimensional_vector2<fp, dim> max(const dimensional_vector2<fp, dim>& a, const dimensional_vector2<fp, dim>& b);
        static dimensional_vector2<fp, dim> min(const dimensional_vector2<fp, dim>& a, const dimensional_vector2<fp, dim>& b);
        inline sixit::units::dimensional_scalar<fp, dim> magnitude() const;
        inline sixit::units::dimensional_scalar<fp, dim * dim> sqr_magnitude() const;
        inline void normalize();
        [[nodiscard]] dimensional_vector2<fp, sixit::units::simple_scalar::dim> normalized() const;
        inline void scale(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> b);
        template <sixit::units::physical_dimension dim_o> 
        inline static dimensional_vector2<fp, dim * dim_o> scale(const dimensional_vector2<fp, dim>& a, sixit::units::dimensional_scalar<fp, dim_o> b);
        /*inline static dimensional_vector2<fp, dim> up();
        inline static dimensional_vector2<fp, dim> down();
        inline static dimensional_vector2<fp, dim> left();
        inline static dimensional_vector2<fp, dim> right();
        inline static dimensional_vector2<fp, dim> one();*/
        inline static dimensional_vector2<fp, dim> zero();

      public:
        inline bool operator==(const dimensional_vector2<fp, dim>& o) const
        {
            return x == o.x && y == o.y;
        }

    //     inline operator vector3<fp>() const;
        inline dimensional_vector2<fp, dim>& operator+=(const dimensional_vector2<fp, dim>& o);
        inline const dimensional_vector2<fp, dim> operator+(const dimensional_vector2<fp, dim>& o) const;
        inline dimensional_vector2<fp, dim>& operator+=(sixit::units::dimensional_scalar<fp, dim> o);
        inline const dimensional_vector2<fp, dim> operator+(sixit::units::dimensional_scalar<fp, dim> o) const;
        inline dimensional_vector2<fp, dim>& operator-=(const dimensional_vector2<fp, dim>& o);
        inline const dimensional_vector2<fp, dim> operator-(const dimensional_vector2<fp, dim>& o) const;
        inline dimensional_vector2<fp, dim>& operator-=(sixit::units::dimensional_scalar<fp, dim> o);
        inline const dimensional_vector2<fp, dim> operator-(sixit::units::dimensional_scalar<fp, dim> o) const;
        inline dimensional_vector2<fp, dim>& operator*=(const dimensional_vector2<fp, sixit::units::simple_scalar::dim>& o);
        template <sixit::units::physical_dimension dim_o>
        inline const dimensional_vector2<fp, dim * dim_o> operator*(const dimensional_vector2<fp, dim_o>& o) const;
        inline dimensional_vector2<fp, dim>& operator*=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);
        template <sixit::units::physical_dimension dim_o>
        inline const dimensional_vector2<fp, dim * dim_o> operator*(const sixit::units::dimensional_scalar<fp, dim_o> o) const;
        inline dimensional_vector2<fp, dim>& operator/=(const dimensional_vector2<fp, sixit::units::simple_scalar::dim>& o);
        template <sixit::units::physical_dimension dim_o>
        inline const dimensional_vector2<fp, dim / dim_o> operator/(const dimensional_vector2<fp, dim_o>& o) const;
        inline dimensional_vector2<fp, dim>& operator/=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);
        template <sixit::units::physical_dimension dim_o>
        inline const dimensional_vector2<fp, dim / dim_o> operator/(const sixit::units::dimensional_scalar<fp, dim_o> o) const;
        inline dimensional_vector2<fp, dim>& operator*=(const matrix4x4<fp>& o);
        inline const dimensional_vector2<fp, dim> operator*(const matrix4x4<fp>& o) const;
        inline dimensional_vector2<fp, dim>& operator*=(const matrix3x3<fp>& o);
        inline const dimensional_vector2<fp, dim> operator*(const matrix3x3<fp>& o) const;
        inline dimensional_vector2<fp, dim>& operator*=(const quaternion<fp>& o);
        inline const dimensional_vector2<fp, dim> operator*(const quaternion<fp>& o) const;
        inline dimensional_vector2<fp, dim> operator-() const;
        inline sixit::units::dimensional_scalar<fp, dim> operator[](int index) const;

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"vector2">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "x", obj.x.value);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "y", obj.y.value);
            sixit::rw::end_struct(comparser);
        }

        [[nodiscard]] inline bool _for_test_only_approximate_eq(const dimensional_vector2<fp, dim>& other, size_t n = 1) const
        {
            return impl::_for_test_only_approximate_eq<fp, fp, 1>(&x.value, &other.x.value, n)
                && impl::_for_test_only_approximate_eq<fp, fp, 1>(&y.value, &other.y.value, n);
        }
    };

    template <typename fp>
    struct vector3: private gpu::vec4p<fp>
    {
        friend struct sixit::lwa::fmt::formatter<direction3<fp>>;
        friend struct sixit::lwa::fmt::formatter<tangent<fp>>;
        friend struct sixit::lwa::fmt::formatter<vector3>;
        
        friend struct sixit::lwa::fmt::formatter<::sixit::geometry::rotation3<fp>>;
        
        friend struct trs3<fp>;
        
        friend struct quaternion<fp>;
        friend struct matrix4x4<fp>;
        friend struct vector2<fp>;
        
        friend struct ray3<fp>;
        
        
        template <typename fp1>
        friend struct tri2_impl;
        friend struct polygon2_impl<fp>;
        friend struct ::sixit::geometry::for_renderers_only::frustum<fp>;
        friend class ::sixit::graphics::normals_buffer_packed_int32;
        friend class ::sixit::graphics::normals_buffer_vec3_float16;
        friend class ::sixit::graphics::tangents_buffer_packed_int32;
        friend class ::sixit::graphics::tangents_buffer_vec4_float16;
        friend class ::sixit::graphics::gpu_vertex;
        friend class ::sixit::graphics::grid;
        friend class ::sixit::geometry::grid3<fp>;
        friend class ::sixit::geometry::grid3_with_bounds<fp>;
        friend class ::sixit::geometry::voxelizer;
        friend class ::sixit::graphics::intersectable_mesh;
        template<typename VoxelDataT> friend class ::sixit::geometry::voxel_grid3;
        friend struct ::sixit::geometry::plane3<fp>;
        friend struct ::sixit::geometry::line_segment3<fp>;
        
        
        friend struct ::sixit::geometry::translation3<fp>;
        friend struct ::sixit::geometry::direction3<fp>;
        friend struct ::sixit::geometry::direction2<fp>;
        
        friend struct ::sixit::geometry::scale3<fp>;
        
        friend struct ::sixit::geometry::rotation2<fp>;
        friend struct ::sixit::geometry::rotation3<fp>;
        friend struct ::sixit::geometry::ray3<fp>;
        friend struct ::sixit::geometry::trs3<fp>;
        friend struct ::sixit::graphics::trs3_animation;
        
        friend struct ::sixit::geometry::projection3to2<fp>;
        friend struct ::sixit::geometry::projection3to2ex<fp>;
        friend struct ::sixit::geometry::projection3to2noscale<fp>;
	    friend struct ::sixit::geometry::projection_perspective3<fp>;
	    friend struct ::sixit::geometry::affine_transform3<fp>;
        friend struct ::sixit::geometry::axis_x_projection<fp>;
        friend struct ::sixit::geometry::axis_y_projection<fp>;
        friend struct ::sixit::geometry::axis_z_projection<fp>;
        
        friend struct ::sixit::geometry::inverse_projection_perspective3<fp>;
        friend class ::sixit::graphics::skeleton_joint;
        
        friend struct ::sixit::geometry::affine_transform3to2<fp>;
        template<typename T> friend class ::sixit::graphics::full_mesh3;
        template<typename fp1, reference_points3<fp1> RefP3ConceptT> friend struct ::sixit::geometry::indexed_point3;

        template <class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T)
        friend class ::sixit::graphics::mesh3_importer;

        template <typename T> friend class ::sixit::graphics::full_mesh3;
        template <typename fp1, reference_points3<fp1> RefP3ConceptT> friend struct ::sixit::geometry::indexed_point3;

        template <typename fp1> friend std::tuple<fp1, fp1, fp1> for_importer_only::xyz_dont_use_outside_of_importer(const point3<fp1>& p);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1, handedness> low_level::for_importer_only::xyzw_dont_use_outside_of_importer(const tangent<fp1>& t);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1, fp1> low_level::for_importer_only::xyzw_dont_use_outside_of_importer(const geometry::rotation3<fp1>& rt3);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_data_transfers_only::xyz_dont_use_outside_of_data_transfers(const point3<fp1>& p);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_data_transfers_only::xyz_dont_use_outside_of_data_transfers(const direction3<fp1>& d);
        template <typename fp1> friend gpu::mat4<fp1> for_renderers_only::matrix_dont_use_outside_of_renderer(const ::sixit::geometry::trs3<fp1>& trs);

        template<typename PCloud> friend ::sixit::geometry::plane3<float>
            (::sixit::geometry::best_fit_plane3)(const PCloud& vlo);

        template<typename VLO> friend ::sixit::geometry::plane3<float>
            (::sixit::geometry::best_fit_plane3_vlo)(const VLO& vlo);

        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void (::sixit::geometry::triangle_voxelize)(const ::sixit::geometry::triangle3& trng, TSetVoxel& SET_VOXEL);
        
        // template <typename TSetVoxel> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::callback_xyz, TSetVoxel)
        // friend void(::sixit::geometry::line_voxelize)(const ::sixit::geometry::line_segment3& ls3, TSetVoxel& SET_VOXEL);

      private:
        static inline const struct position_type_t {} position_type = {};
        static inline const struct normal_type_t {} normal_type = {};
        inline vector3(const gpu::vec4p<fp>& o);
        inline vector3(const gpu::vec4base<fp>& o);
        inline vector3();
        inline vector3(fp x, fp y, fp z);
        inline vector3(fp x, fp y, fp z, position_type_t);
        inline vector3(fp x, fp y, fp z, normal_type_t);
        inline vector3(fp x, fp y, fp z, fp w);
        inline vector3(const vector2<fp>& o);
        inline void set(fp x, fp y, fp z);
        static fp dot(const vector3& a, const vector3& b);
        static vector3 cross(const vector3& a, const vector3& b);
        inline static fp angle(const vector3& a, const vector3& b);
        static vector3 max(const vector3& a, const vector3& b);
        static vector3 min(const vector3& a, const vector3& b);
        inline fp magnitude() const;
        inline fp sqr_magnitude() const;
        inline void normalize();
        [[nodiscard]] inline vector3 normalized() const;
        inline void scale(fp b);
        inline static vector3 scale(const vector3& a, fp b);
        /*inline static vector3 forward();
        inline static vector3 back();
        inline static vector3 up();
        inline static vector3 down();
        inline static vector3 left();
        inline static vector3 right();
        inline static vector3 one();*/
        inline static vector3 zero();

        [[nodiscard]] inline bool _for_test_only_approximate_eq(const vector3<fp>& other, size_t n = 1) const
        {
            return impl::_for_test_only_approximate_eq<fp, 3>(vector3<fp>::const_ptr(), other.const_ptr(), n);
        }

      public:
        inline bool operator==(const vector3<fp>& o) const;
        inline operator vector2<fp>() const;
        inline vector3& operator+=(const vector3<fp>& o);
        inline const vector3 operator+(const vector3<fp>& o) const;
        inline vector3& operator+=(fp o);
        inline const vector3 operator+(fp o) const;
        inline vector3& operator-=(const vector3<fp>& o);
        inline const vector3 operator-(const vector3<fp>& o) const;
        inline vector3& operator-=(fp o);
        inline const vector3 operator-(fp o) const;
        inline vector3& operator*=(const vector3<fp>& o);
        inline const vector3 operator*(const vector3<fp>& o) const;
        inline vector3& operator*=(fp o);
        inline const vector3 operator*(fp o) const;
        inline vector3& operator/=(const vector3<fp>& o);
        inline const vector3 operator/(const vector3<fp>& o) const;
        inline vector3& operator/=(fp o);
        inline const vector3 operator/(fp o) const;
        inline vector3& operator*=(const matrix4x4<fp>& o);
        inline const vector3 operator*(const matrix4x4<fp>& o) const;
        inline vector3& operator*=(const quaternion<fp>& o);
        inline const vector3 operator*(const quaternion<fp>& o) const;

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"vector3">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "x", obj.x());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "y", obj.y());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "z", obj.z());
            sixit::rw::end_struct(comparser);
        }
    };

    template <typename fp, sixit::units::physical_dimension dim>
    struct dimensional_vector3: private gpu::vec4p<fp>
    {
        using element_t = sixit::units::dimensional_scalar<fp, dim>;

        template <typename fp1, sixit::units::physical_dimension dim1>
        friend struct dimensional_vector3;
        friend struct sixit::lwa::fmt::formatter<direction3<fp>>;
        friend struct ::sixit::geometry::scale3<fp>;
        friend struct ::sixit::geometry::point3<fp>;
        friend struct ::sixit::geometry::translation3<fp>;
        friend struct ::sixit::geometry::direction3<fp>;
        friend class ::sixit::graphics::skeleton_joint;

        friend struct sixit::lwa::fmt::formatter<::sixit::geometry::rotation3<fp>>;
        friend struct sixit::lwa::fmt::formatter<::sixit::geometry::scale3<fp>>;
        friend struct sixit::lwa::fmt::formatter<::sixit::geometry::translation3<fp>>;

        friend struct plane3_impl<fp>;
        friend struct ls3_impl<fp>;
        friend struct tri3_impl<fp>;
        friend struct ::sixit::geometry::plane3<fp>;

        friend struct ::sixit::geometry::triangle3<fp>;
        friend struct ::sixit::geometry::bounds3<fp>;
        friend struct ::sixit::geometry::barycentric<fp>;
        template <typename fp1>
        friend struct ::sixit::geometry::affine_transform3;
        friend struct ::sixit::geometry::affine_transform3to2<fp>;
        friend struct ::sixit::geometry::mirror_transform3<fp>;
        friend struct ::sixit::geometry::mirror_transform2<fp>;
        friend struct ::sixit::geometry::rotation2<fp>;
        friend struct ::sixit::geometry::rotation3<fp>;
        friend struct ::sixit::geometry::indexed_point3_cloud<fp>;
        friend struct ::sixit::geometry::projection3<fp>;
        friend struct ::sixit::geometry::point3_cloud<fp>;

        friend struct quaternion<fp>;

        template <typename fp1, sixit::units::physical_dimension dim1>
        friend struct dimensional_quaternion;

        friend class ::sixit::graphics::gpu_vertex;
        friend struct graphics::trs3_animation;        

        friend struct ::sixit::geometry::projection3to2<fp>;
        friend struct ::sixit::geometry::projection3to2ex<fp>;
        friend struct ::sixit::geometry::projection2to1ex<fp>;
        friend struct ::sixit::geometry::projection3to2noscale<fp>;

        friend class ::sixit::graphics::normals_buffer_packed_int32;

        friend class ::sixit::graphics::normals_buffer_vec3_float16;
        friend class ::sixit::graphics::tangents_buffer_packed_int32;
        friend class ::sixit::graphics::tangents_buffer_vec4_float16;

        friend struct ::sixit::geometry::axis_x_projection<fp>;
        friend struct ::sixit::geometry::axis_y_projection<fp>;
        friend struct ::sixit::geometry::axis_z_projection<fp>;

        friend struct ::sixit::geometry::tangent<fp>;
        friend struct ::sixit::geometry::inverse_projection_perspective3<fp>;
        friend struct ::sixit::geometry::for_renderers_only::frustum<fp>;
        friend struct ::sixit::geometry::trs3<fp>;

        friend struct ::sixit::geometry::projection_perspective3<fp>;

        friend class ::sixit::geometry::grid3<fp>;
        friend class ::sixit::geometry::voxelizer;

        template <class T> SIXIT_LWA_OPTIONAL_REQUIRES(::sixit::geometry::full_mesh_concept, T)
        friend class ::sixit::graphics::mesh3_importer;
        
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> for_renderers_only::xyz_dont_use_outside_of_renderer(const point3<fp1>& p);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1, handedness> low_level::for_importer_only::xyzw_dont_use_outside_of_importer(const tangent<fp1>& t);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> for_importer_only::xyz_dont_use_outside_of_importer(const point3<fp1>& p);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_importer_only::xyz_dont_use_outside_of_importer(const geometry::translation3<fp1>& tr3);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> low_level::for_importer_only::xyz_dont_use_outside_of_importer(const geometry::scale3<fp1>& sc3);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> for_renderers_only::xyz_dont_use_outside_of_renderer(const direction3<fp1>& d);
        template <typename fp1> friend std::tuple<fp1, fp1, fp1> for_renderers_only::xyz_dont_use_outside_of_renderer(const tangent<fp1>& t);
        template <typename fp1> friend gpu::mat4<fp1> for_renderers_only::matrix_dont_use_outside_of_renderer(const ::sixit::geometry::trs3<fp1>& trs);

        template<typename PCloud> friend ::sixit::geometry::plane3<float>
            (::sixit::geometry::best_fit_plane3)(const PCloud& vlo);

        template<typename VLO> friend ::sixit::geometry::plane3<float>
            (::sixit::geometry::best_fit_plane3_vlo)(const VLO& vlo);

      private:
        static inline const struct position_type_t {} position_type = {};
        static inline const struct normal_type_t {} normal_type = {};
        inline dimensional_vector3(const gpu::vec4p<fp>& o);
        inline dimensional_vector3(const gpu::vec4base<fp>& o);
        inline dimensional_vector3();
        inline dimensional_vector3(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y, 
                                    sixit::units::dimensional_scalar<fp, dim> z);
        inline dimensional_vector3(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y, 
                        sixit::units::dimensional_scalar<fp, dim> z, position_type_t);
        inline dimensional_vector3(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y, 
                        sixit::units::dimensional_scalar<fp, dim> z, normal_type_t);
        inline dimensional_vector3(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y, 
                        sixit::units::dimensional_scalar<fp, dim> z, sixit::units::dimensional_scalar<fp, dim> w);
        inline dimensional_vector3(const dimensional_vector2<fp, dim>& o);
        inline void set(sixit::units::dimensional_scalar<fp, dim> x, sixit::units::dimensional_scalar<fp, dim> y, 
                        sixit::units::dimensional_scalar<fp, dim> z);
        template <sixit::units::physical_dimension dim1>
        static sixit::units::dimensional_scalar<fp, dim * dim1> dot(const dimensional_vector3<fp, dim>& a, const dimensional_vector3<fp, dim1>& b);
        template <sixit::units::physical_dimension dim1>
        static dimensional_vector3<fp, dim * dim1> cross(const dimensional_vector3<fp, dim>& a, const dimensional_vector3<fp, dim1>& b);
        inline static sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> angle(
                        const dimensional_vector3& a, const dimensional_vector3<fp, dim>& b);
        static dimensional_vector3<fp, dim> max(const dimensional_vector3<fp, dim>& a, const dimensional_vector3<fp, dim>& b);
        static dimensional_vector3<fp, dim> min(const dimensional_vector3<fp, dim>& a, const dimensional_vector3<fp, dim>& b);
        inline sixit::units::dimensional_scalar<fp, dim> magnitude() const;
        inline sixit::units::dimensional_scalar<fp, dim * dim> sqr_magnitude() const;
        [[nodiscard]] inline dimensional_vector3<fp, sixit::units::simple_scalar::dim> normalized() const;
        inline void scale(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> b);
        template <sixit::units::physical_dimension dim1>
        inline static dimensional_vector3<fp, dim * dim1> scale(const dimensional_vector3<fp, dim>& a, sixit::units::dimensional_scalar<fp, dim1> b);
        /*inline static dimensional_vector3<fp, dim> forward();
        inline static dimensional_vector3<fp, dim> back();
        inline static dimensional_vector3<fp, dim> up();
        inline static dimensional_vector3<fp, dim> down();
        inline static dimensional_vector3<fp, dim> left();
        inline static dimensional_vector3<fp, dim> right();
        inline static dimensional_vector3<fp, dim> one();*/
        inline static dimensional_vector3<fp, dim> zero();

        inline sixit::units::dimensional_scalar<fp, dim> dim_x() const {
            return sixit::units::dimensional_scalar<fp, dim>({ dimensional_vector3<fp, dim>::x(), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() });
        }
        inline sixit::units::dimensional_scalar<fp, dim> dim_y() const {
            return sixit::units::dimensional_scalar<fp, dim>({ dimensional_vector3<fp, dim>::y(), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() });
        }
        inline sixit::units::dimensional_scalar<fp, dim> dim_z() const {
            return sixit::units::dimensional_scalar<fp, dim>({ dimensional_vector3<fp, dim>::z(), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() });
        }

        [[nodiscard]] inline bool _for_test_only_approximate_eq(const dimensional_vector3<fp, dim>& other, size_t n = 1) const
        {
            return impl::_for_test_only_approximate_eq<fp, fp, 3>(dimensional_vector3<fp, dim>::const_ptr(), other.const_ptr(), n);
        }

      public:
        inline bool operator==(const dimensional_vector3<fp, dim>& o) const;
        inline operator dimensional_vector2<fp, dim>() const;
        inline dimensional_vector3<fp, dim>& operator+=(const dimensional_vector3<fp, dim>& o);
        inline const dimensional_vector3<fp, dim> operator+(const dimensional_vector3<fp, dim>& o) const;
        inline dimensional_vector3<fp, dim>& operator+=(sixit::units::dimensional_scalar<fp, dim> o);
        inline const dimensional_vector3<fp, dim> operator+(sixit::units::dimensional_scalar<fp, dim> o) const;
        inline dimensional_vector3<fp, dim>& operator-=(const dimensional_vector3<fp, dim>& o);
        inline const dimensional_vector3<fp, dim> operator-(const dimensional_vector3<fp, dim>& o) const;
        inline dimensional_vector3<fp, dim>& operator-=(sixit::units::dimensional_scalar<fp, dim> o);
        inline const dimensional_vector3<fp, dim> operator-(sixit::units::dimensional_scalar<fp, dim> o) const;
        inline dimensional_vector3<fp, dim>& operator*=(const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& o);
        template <sixit::units::physical_dimension dim1>
        inline const dimensional_vector3<fp, dim * dim1> operator*(const dimensional_vector3<fp, dim1>& o) const;
        inline dimensional_vector3<fp, dim>& operator*=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);
        template <sixit::units::physical_dimension dim1>
        inline const dimensional_vector3<fp, dim * dim1> operator*(sixit::units::dimensional_scalar<fp, dim1> o) const;
        inline dimensional_vector3<fp, dim>& operator/=(const dimensional_vector3<fp, sixit::units::simple_scalar::dim>& o);
        template <sixit::units::physical_dimension dim1>
        inline const dimensional_vector3<fp, dim / dim1> operator/(const dimensional_vector3<fp, dim1>& o) const;
        inline dimensional_vector3<fp, dim>& operator/=(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> o);
        template <sixit::units::physical_dimension dim1>
        inline const dimensional_vector3<fp, dim / dim1> operator/(sixit::units::dimensional_scalar<fp, dim1> o) const;
        inline dimensional_vector3<fp, dim>& operator*=(const matrix4x4<fp>& o);
        inline const dimensional_vector3<fp, dim> operator*(const matrix4x4<fp>& o) const;
        inline dimensional_vector3<fp, dim>& operator*=(const dimensional_quaternion<fp, sixit::units::simple_scalar::dim>& o);
        template <sixit::units::physical_dimension dim1>
        inline const dimensional_vector3<fp, dim * dim1> operator*(const dimensional_quaternion<fp, dim1>& o) const;

        template <typename This, typename ComparserT>
        static void read_write(This& obj, ComparserT& comparser)
        {
            sixit::rw::begin_struct<"vector3">(comparser, obj);
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "x", obj.x());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "y", obj.y());
            sixit::rw::read_write_runtime_name_slower_than_complie_time_name<sixit::rw::F32>(comparser, "z", obj.z());
            sixit::rw::end_struct(comparser);
        }
    };
}; // namespace low_level

namespace low_level 
{
    // aliases 
    using vector2f = vector2<float>;
    template <sixit::units::physical_dimension dim>
    using dimensional_vector2f = dimensional_vector2<float, dim>;

    using vector3f = vector3<float>;
    template <sixit::units::physical_dimension dim>
    using dimensional_vector3f = dimensional_vector3<float, dim>;
}


}; // namespace geometry
}; // namespace sixit

#endif //sixit_geometry_low_level_vector_h_included

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
