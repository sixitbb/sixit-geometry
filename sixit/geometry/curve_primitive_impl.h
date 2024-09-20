/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_curve_primitive_impl_h_included
#define sixit_geometry_curve_primitive_impl_h_included

#include <memory>

#include "sixit/geometry/curve_primitive.h"
#include "sixit/geometry/arc.h"
#include "sixit/geometry/line.h"
#include "sixit/geometry/qbezier.h"
#include "sixit/geometry/cbezier.h"
#include "sixit/geometry/curve.h"
#include "sixit/geometry/shapes.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/core/guidelines.h"
namespace sixit
{
	namespace geometry
	{
		template <typename fp>
		inline std::pair<std::optional<sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>>, std::optional<sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim>>> 
		solve_quadric_equation(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> a, 
								sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> b, 
									sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> c)
		{
			auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
			auto fp_4 = sixit::units::create_dimensionless_scalar<fp>(4.f);

			auto d = (b * b - fp_4 * a * c);
			if (d < decltype(d)::zero())
				return { {}, {} };

			auto sr_d = low_level::mathf::sqrt(d);
			auto x1 = (-b - sr_d) / (fp_2 * a);
			auto x2 = (-b + sr_d) / (fp_2 * a);
			if (sixit::geometry::low_level::mathf::isfinite(x1) && sixit::geometry::low_level::mathf::isfinite(x2))
			{
				if (x1 == x2)
					return { {x1}, {} };
				else
					return { {x1}, { x2 } };
			}
			else
			{
				x1 = -c / b;
				if (sixit::geometry::low_level::mathf::isfinite(x1))
					return { {x1}, {} };
				else
					return { {}, {} };
			}
		}

		// arc2 implementation
		template <typename fp>
		bool arc2<fp>::operator==(const arc2<fp>& o) const 
		{
			return center.vec() == o.center.vec() && radius == o.radius && angle0 == o.angle0 && angle1 == o.angle1;
		}

		template <typename fp>
		typename arc2<fp>::class_value arc2<fp>::length() const 
		{
			return low_level::mathf::abs(angle1 - angle0) * radius;
		}

		template <typename fp>
		point2<fp> arc2<fp>::track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
		{
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
			auto a = angle0 * (fp_1 - t) + angle1 * t;
		return point2(center.vec() + low_level::dimensional_vector2(radius * low_level::mathf::cos(a), radius * low_level::mathf::sin(a)));
		}
		
		template <typename fp>
		direction2<fp> arc2<fp>::track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
		{
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
			auto a = angle0 * (fp_1 - t) + angle1 * t;
			return angle1 > angle0 ? direction2(-low_level::mathf::sin(a), low_level::mathf::cos(a)) : direction2(low_level::mathf::sin(a), -low_level::mathf::cos(a));
		}

		template <typename fp>
		bounds2<fp> arc2<fp>::bounds() const
		{
			auto fp_pi = sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::pi);
			auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);

			static const auto half_pi = fp_pi / fp_2;

			low_level::dimensional_vector2 p0 = center.vec() + low_level::dimensional_vector2(radius * low_level::mathf::cos(angle0), radius * low_level::mathf::sin(angle0));
			low_level::dimensional_vector2 p1 = center.vec() + low_level::dimensional_vector2(radius * low_level::mathf::cos(angle1), radius * low_level::mathf::sin(angle1));
			low_level::dimensional_vector2 min = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::min(p0, p1);
			low_level::dimensional_vector2 max = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::max(p0, p1);

			if (is_in_arc(fp_0))
				max = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::max(max, low_level::dimensional_vector2(center.vec().x + radius, center.vec().y));
			if (is_in_arc(fp_pi))
				min = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::min(min, low_level::dimensional_vector2(center.vec().x - radius, center.vec().y));
			if (is_in_arc(half_pi))
				max = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::max(max, low_level::dimensional_vector2(center.vec().x, center.vec().y + radius));
			if (is_in_arc(-half_pi))
				min = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::min(min, low_level::dimensional_vector2(center.vec().x, center.vec().y - radius));

			return bounds2<fp>(min, max);
		}

		template <typename fp>
		void arc2<fp>::intersection(typename std::vector<point2<fp>>& intersections, const typename curve2_primitive<fp>::pcurve_segment& other) const
		{
			return other->intersection(intersections, *this);
		}

		template <typename fp>
		void arc2<fp>::intersection(std::vector<point2<fp>>& intersections, const line2<fp>& other) const
		{
			low_level::dimensional_vector2 line_dir = (other.p2().vec() - other.p1().vec()).normalized();
			low_level::dimensional_vector2 line_rn(line_dir.y, -line_dir.x);
			low_level::dimensional_vector2 rad_vec = other.p1().vec() - center.vec();
			auto d = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(rad_vec, line_rn);
			if (low_level::mathf::abs(d) > radius)
				return;

			low_level::dimensional_vector2 mid_chord = center.vec() + line_rn * d;
			auto o = low_level::mathf::sqrt(radius * radius - d * d);
			low_level::dimensional_vector2 ip_0 = mid_chord + line_dir * o;
			if (is_in_arc(angle_from_radius_vector(direction2<fp>(ip_0 - center.vec()).vec())))
			{
				intersections.push_back(point2(ip_0));
			}

			low_level::dimensional_vector2 ip_1 = mid_chord - line_dir * o;
			if (ip_1 != ip_0 && is_in_arc(angle_from_radius_vector(direction2<fp>(ip_1 - center.vec()).vec())))
			{
				intersections.push_back(point2(ip_1));
			}
		}

		template <typename fp>
		void arc2<fp>::intersection(std::vector<point2<fp>>& intersections, const line_segment2<fp>& other) const
		{
			auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f);
			low_level::dimensional_vector2 line_dir = (other.p2().vec() - other.p1().vec()).normalized();
			low_level::dimensional_vector2 line_rn(line_dir.y, -line_dir.x);

			low_level::dimensional_vector2 rad_vec = other.p1().vec() - center.vec();
			auto d = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(rad_vec, line_rn);
			if (low_level::mathf::abs(d) > radius)
				return;

			low_level::dimensional_vector2 mid_chord = center.vec() + line_rn * d;
			auto o = low_level::mathf::sqrt(radius * radius - d * d);
			auto mid_l = -low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(rad_vec, line_dir);
			auto line_seg_lenght = other.length();

			// ip0
			auto l_0 = mid_l + o;
			low_level::dimensional_vector2 ip_0 = mid_chord + line_dir * o;
			if (l_0 >= fp_0 && l_0 <= line_seg_lenght)
			{
				if (is_in_arc(angle_from_radius_vector(direction2<fp>(ip_0 - center.vec()).vec())))
				{
					intersections.push_back(point2(ip_0));
				}
			}

			// ip1
			auto l_1 = mid_l - o;
			low_level::dimensional_vector2 ip_1 = mid_chord - line_dir * o;
			if (ip_1 != ip_0 && l_1 >= fp_0 && l_1 <= line_seg_lenght)
			{
				if (is_in_arc(angle_from_radius_vector(direction2<fp>(ip_1 - center.vec()).vec())))
				{
					intersections.push_back(point2(ip_1));
				}
			}
		}

		template <typename fp>
		void arc2<fp>::intersection(std::vector<point2<fp>>& intersections, const arc2& other) const
		{
			auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
			low_level::dimensional_vector2 c0_c1 = other.center.vec() - center.vec();
			auto l_c0_c1 = c0_c1.magnitude();
			if ((l_c0_c1 > radius + other.radius) || (l_c0_c1 < low_level::mathf::abs(radius - other.radius)) || (l_c0_c1 <= decltype(l_c0_c1)::zero()))
				return;

			auto aa = (radius * radius - other.radius * other.radius + l_c0_c1 * l_c0_c1) / (fp_2 * l_c0_c1);
			low_level::dimensional_vector2 m = center.vec() + c0_c1 * aa / l_c0_c1;
			auto sq_hh = radius * radius - aa * aa;

			low_level::dimensional_vector2 n(c0_c1.y, -c0_c1.x);
			auto hh = low_level::mathf::sqrt(sq_hh) / l_c0_c1;
			low_level::dimensional_vector2 p1 = m + n * hh;
			if (point_in_both_arcs(p1, other))
			{
				intersections.push_back(point2(p1));
			}
			low_level::dimensional_vector2 p2 = m - n * hh;
			if (p2 != p1 && point_in_both_arcs(p2, other))
			{
				intersections.push_back(point2(p2));
			}
		}

		template <typename fp>
		void arc2<fp>::intersection(std::vector<point2<fp>>& intersections, const qbezier2<fp>& other) const { other.intersection(intersections, *this); }

		template <typename fp>
		void arc2<fp>::intersection(std::vector<point2<fp>>& intersections, const cbezier2<fp>& other) const { other.intersection(intersections, *this); }

		template <typename fp>
		bool arc2<fp>::intersect(const typename curve2_primitive<fp>::pcurve_segment& other) const { return other->intersect(*this); }

		template <typename fp>
		bool arc2<fp>::intersect(const line2<fp>& other) const
		{
			low_level::dimensional_vector2 line_dir = (other.p2().vec() - other.p1().vec()).normalized();
			low_level::dimensional_vector2 line_rn(line_dir.y, -line_dir.x);
			low_level::dimensional_vector2 rad_vec = other.p1().vec() - center.vec();
			auto d = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(rad_vec, line_rn);
			
			if (low_level::mathf::abs(d) > radius)
				return false;

			low_level::dimensional_vector2 mid_chord = center.vec() + line_rn * d;
			auto o = low_level::mathf::sqrt(radius * radius - d * d);
			low_level::dimensional_vector2 ip_0 = mid_chord + line_dir * o;
			
			auto r_vec0 = (ip_0 - center.vec());
			if (is_in_arc(angle_from_radius_vector(direction2<fp>(r_vec0).vec())))
				return true;

			low_level::dimensional_vector2 ip_1 = mid_chord - line_dir * o;
			auto r_vec1 = (ip_1 - center.vec());
			return is_in_arc(angle_from_radius_vector(direction2<fp>(r_vec1).vec()));
		}

		template <typename fp>
		bool arc2<fp>::intersect(const line_segment2<fp>& other) const
		{
			low_level::dimensional_vector2 line_dir = (other.p2().vec() - other.p1().vec()).normalized();
			low_level::dimensional_vector2 line_rn(line_dir.y, -line_dir.x);

			low_level::dimensional_vector2 rad_vec = other.p1().vec() - center.vec();
			auto d = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(rad_vec, line_rn);
			if (low_level::mathf::abs(d) > radius)
				return false;

			low_level::dimensional_vector2 mid_chord = center.vec() + line_rn * d;
			auto o = low_level::mathf::sqrt(radius * radius - d * d);
			auto mid_l = -low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(rad_vec, line_dir);
			auto line_seg_lenght = other.length();

			// ip0
			auto l_0 = mid_l + o;
			if (l_0 >= decltype(l_0)::zero() && l_0 <= line_seg_lenght)
			{
				low_level::dimensional_vector2 ip_0 = mid_chord + line_dir * o;
				auto r_vec = (ip_0 - center.vec());
				if (is_in_arc(angle_from_radius_vector(direction2<fp>(r_vec).vec())))
					return true;
			}

			// ip1
			auto l_1 = mid_l - o;
			if (l_1 >= decltype(l_1)::zero() && l_1 <= line_seg_lenght)
			{
				low_level::dimensional_vector2 ip_1 = mid_chord - line_dir * o;
				auto r_vec = (ip_1 - center.vec());
				return is_in_arc(angle_from_radius_vector(direction2<fp>(r_vec).vec()));
			}
			return false;
		}

		template <typename fp>
		bool arc2<fp>::intersect(const arc2& other) const
		{
			auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
			low_level::dimensional_vector2 c0_c1 = other.center.vec() - center.vec();
			auto l_c0_c1 = c0_c1.magnitude();
			if ((l_c0_c1 > radius + other.radius) || (l_c0_c1 < low_level::mathf::abs(radius - other.radius)) || (l_c0_c1 <= decltype(l_c0_c1)::zero()))
				return false;

			auto aa = (radius * radius - other.radius * other.radius + l_c0_c1 * l_c0_c1) / (fp_2 * l_c0_c1);
			low_level::dimensional_vector2 m = center.vec() + c0_c1 * aa / l_c0_c1;
			auto sq_hh = radius * radius - aa * aa;

			low_level::dimensional_vector2 n(c0_c1.y, -c0_c1.x);
			auto hh = low_level::mathf::sqrt(sq_hh) / l_c0_c1;
			if (point_in_both_arcs(m + n * hh, other))
				return true;
			if (point_in_both_arcs(m - n * hh, other))
				return true;

			return false;
		}

		template <typename fp>
		bool arc2<fp>::intersect(const qbezier2<fp>& other) const { return other.intersect(*this); }

		template <typename fp>
		bool arc2<fp>::intersect(const cbezier2<fp>& other) const { return other.intersect(*this); }

		template <typename fp>
		bool arc2<fp>::point_in_both_arcs(const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p, const arc2& other) const
		{
			auto r_vec0 = p - center.vec();
			auto r_vec1 = p - other.center.vec();
			return (is_in_arc(angle_from_radius_vector(direction2<fp>(r_vec0).vec())) && other.is_in_arc(angle_from_radius_vector(direction2<fp>(r_vec1).vec())));
		}

		template <typename fp>
		template<typename TSetPixel>
		inline void arc2<fp>::rasterize(TSetPixel& SET_PIXEL, class_value dist_between_points) const
		{
			const fp point_count = low_level::mathf::max(fp(4.f), low_level::mathf::floor(
				sixit::units::extract_dim_less_scalar(length() / dist_between_points))); // min 4 points
			fp delta = fp(1.0f) / point_count;

			point2<fp> p1, p0 = track_point(sixit::units::create_dimensionless_scalar<fp>(0.f));
			for (fp i = fp(1.f); i <= point_count; i = i + fp(1.f))
			{
				p1 = track_point(sixit::units::create_dimensionless_scalar<fp>(delta * i));
				line_segment2<fp>::rasterize(p0, p1, SET_PIXEL);
				p0 = p1;
			}
		}

		template <typename fp>
		template<typename TSetPixelAA>
        inline void arc2<fp>::rasterize_aa(TSetPixelAA& SET_PIXEL_AA, class_value dist_between_points) const
		{
			const fp point_count = low_level::mathf::max(fp(4.f), 
			sixit::units::extract_dim_less_scalar(low_level::mathf::floor(length() / dist_between_points))); // min 4 points
			fp delta = fp(1.0f) / point_count;

			point2<fp> p1, p0 = track_point(sixit::units::create_dimensionless_scalar<fp>(0.f));
			for (fp i = fp(1.f); i <= point_count; i = i + fp(1.f))
			{
				p1 = track_point(sixit::units::create_dimensionless_scalar<fp>(delta * i));
				line_segment2<fp>::rasterize_aa(p0, p1, SET_PIXEL_AA);
				p0 = p1;
			}
		}

		// qbezier2 implementation
		template <typename fp>
		bool qbezier2<fp>::operator==(const qbezier2<fp>& o) const
		{
			return  (qbezier2<fp>::p1().vec() == o.p1().vec()) &&
				(qbezier2<fp>::p2().vec() == o.p2().vec()) &&
				(qbezier2<fp>::p3().vec() == o.p3().vec());
		}

		template <typename fp>
		template <typename fp2>
		bool qbezier2<fp>::operator==(const qbezier2<fp2>& o) const
		{
			return (this->p1() == o.p1()) &&
				(this->p2() == o.p2()) &&
				(this->p3() == o.p3());
		}

		template <typename fp>
		typename qbezier2<fp>::class_value qbezier2<fp>::_length() const
		{
			// in track_value we use equation t**2 * P1 + 2 * t * (1-t) * P2 + (1-t)**2 * P2
			// but here for calculating the length we use equation form A * t**2 + B * t + C
			// L = integral (0,t) sqrt(A * t**2 + B * t + C) dt
			// then b = B/(2*C*A), c = C/A   ==>  L = sqrt(A) * integral (0,t) sqrt(t**2 + 2b * t + c) dt
			// then u = t + b, k = c - b**2  ==>  L = sqrt(A) * integral (b,u) sqrt(u**2 + k) dt
			fp fp_05(0.5f);
			fp fp_1(1.f);
			fp fp_2(2.f);
			fp fp_4(4.f);

			auto segments_length = [&]() { 
				auto mp = track_point(sixit::units::create_dimensionless_scalar<fp>(fp_05)); 
				return (mp.vec() - qbezier2<fp>::p1().vec()).magnitude() + (qbezier2<fp>::p3().vec() - mp.vec()).magnitude(); };

			low_level::dimensional_vector2 p12 = qbezier2<fp>::p1().vec() - qbezier2<fp>::p2().vec();
			low_level::dimensional_vector2 p32 = qbezier2<fp>::p3().vec() - qbezier2<fp>::p2().vec();
			auto cross = p12.x * p32.y - p12.y * p32.x;

			low_level::dimensional_vector2 aa = p12 + p32;	// P1 - 2*P2 + P3
			low_level::dimensional_vector2 bb = -p12 * sixit::units::create_dimensionless_scalar<fp>(fp_2);	// 2*P2 - 2*P1

			auto A = sixit::units::create_dimensionless_scalar<fp>(fp_4) * aa.sqr_magnitude();				// ((aa.x * aa.x) + (aa.y * aa.y));
			auto B = sixit::units::create_dimensionless_scalar<fp>(fp_4) * ((aa.x * bb.x) + (aa.y * bb.y));
			auto C = bb.sqr_magnitude();					// (bb.x * bb.x) + (bb.y * bb.y);
			auto b = B / (sixit::units::create_dimensionless_scalar<fp>(fp_2) * A);
			auto c = C / A;

			if (!sixit::geometry::low_level::mathf::isfinite(b) || !sixit::geometry::low_level::mathf::isfinite(c))
			{
				return segments_length();
			}

			auto u = sixit::units::create_dimensionless_scalar<fp>(fp_1) + b;
			auto k = c - (b * b);

			// final equation of integral
			auto mm = sixit::units::create_dimensionless_scalar<fp>(fp_05) * low_level::mathf::sqrt(A);
			auto uuk = low_level::mathf::sqrt((u * u) + k);
			auto bbk = low_level::mathf::sqrt((b * b) + k);
			auto nom = u + uuk;
			auto denom = b + bbk;

			auto resnom = nom / denom;
			if (!sixit::geometry::low_level::mathf::isfinite(resnom))
			{
				return segments_length();
			}

			auto lg = sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::log(sixit::units::extract_dim_less_scalar(low_level::mathf::abs(resnom))));
			auto ll = mm * (u * uuk - b * bbk + k * lg);
			return ll;
		}

		template <typename fp>
		point2<fp> qbezier2<fp>::track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
		{
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
			auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
			auto inv_t = fp_1 - t;
			return point2(qbezier2<fp>::p1().vec() * inv_t * inv_t + qbezier2<fp>::p2().vec() * fp_2 * inv_t * t + qbezier2<fp>::p3().vec() * t * t);
		}

		template <typename fp>
		direction2<fp> qbezier2<fp>::track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
		{
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
			auto inv_t = fp_1 - t;
			return direction2<fp>((qbezier2<fp>::p2().vec() - qbezier2<fp>::p1().vec()) * inv_t + 
									(qbezier2<fp>::p3().vec() - qbezier2<fp>::p2().vec()) * t);
		}

		template <typename fp>
		bounds2<fp> qbezier2<fp>::bounds() const
		{
			// make bound from end points
			low_level::dimensional_vector2 min = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::min(qbezier2<fp>::p1().vec(), qbezier2<fp>::p3().vec());
			low_level::dimensional_vector2 max = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::max(qbezier2<fp>::p1().vec(), qbezier2<fp>::p3().vec());

			// check x
			auto extr_x = qbezier2<fp>::p2().vec().x;
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
			auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
			if (extr_x < min.x || extr_x > max.x)
			{
				auto t = low_level::mathf::clamp01((qbezier2<fp>::p1().vec().x - qbezier2<fp>::p2().vec().x) / (qbezier2<fp>::p1().vec().x - fp_2 * qbezier2<fp>::p2().vec().x + qbezier2<fp>::p3().vec().x));
				auto s = fp_1 - t;
				extr_x = s * s * qbezier2<fp>::p1().vec().x + fp_2 * s * t * qbezier2<fp>::p2().vec().x + t * t * qbezier2<fp>::p3().vec().x;
			}

			// check y
			auto extr_y = qbezier2<fp>::p2().vec().y;
			if (extr_y < min.y || extr_y > max.y)
			{
				auto t = low_level::mathf::clamp01((qbezier2<fp>::p1().vec().y - qbezier2<fp>::p2().vec().y) / (qbezier2<fp>::p1().vec().y - fp_2 * qbezier2<fp>::p2().vec().y + qbezier2<fp>::p3().vec().y));
				auto s = fp_1 - t;
				extr_y = s * s * qbezier2<fp>::p1().vec().y + fp_2 * s * t * qbezier2<fp>::p2().vec().y + t * t * qbezier2<fp>::p3().vec().y;
			}

			max = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::max(max, low_level::dimensional_vector2(extr_x, extr_y));
			min = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::min(min, low_level::dimensional_vector2(extr_x, extr_y));
			return bounds2<fp>(min, max);
		}

		template <typename fp>
		typename qbezier2<fp>::class_value qbezier2<fp>::min_curvature_radius() const
		{
			fp fp_0(0.f);
			fp fp_2(2.f);
			fp fp_inf(low_level::mathf::infinity);
			fp fp_05(0.5f);
			fp fp_16(16.f);

			low_level::dimensional_vector2 p10 = qbezier2<fp>::p2().vec() - qbezier2<fp>::p1().vec();
			low_level::dimensional_vector2 p21 = qbezier2<fp>::p3().vec() - qbezier2<fp>::p2().vec();
			auto area = low_level::mathf::abs(p10.x * p21.y - p10.y * p21.x) / sixit::units::create_dimensionless_scalar<fp>(fp_2);
			if (area == decltype(area)::zero())
				return low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(p10, p21) > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(fp_0) 
					? sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_inf) 
					: sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_0);

			low_level::dimensional_vector2 m = (qbezier2<fp>::p3().vec() + qbezier2<fp>::p1().vec()) * sixit::units::create_dimensionless_scalar<fp>(fp_05);

			low_level::dimensional_vector2 chord = qbezier2<fp>::p3().vec() - qbezier2<fp>::p1().vec();
			auto dd = chord.sqr_magnitude() / sixit::units::create_dimensionless_scalar<fp>(fp_16);	// (chord / 4) ^ 2 - distance for check in special cases

			// check special cases
			low_level::dimensional_vector2 p00_2 = qbezier2<fp>::p2().vec() - ((m + qbezier2<fp>::p1().vec()) * sixit::units::create_dimensionless_scalar<fp>(fp_05));
			if (p00_2.sqr_magnitude() <= dd)
			{
				// p1 close to p0 - max curvature in p0
				auto p10_l = p10.magnitude();
				auto res = p10_l * p10_l * p10_l / area; 
				return sixit::geometry::low_level::mathf::isfinite(res) ? res :
					(low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(p10, p21) > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(fp_0)
					? sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_inf)
					: sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_0));
			}

			low_level::dimensional_vector2 p0_22 = qbezier2<fp>::p2().vec() - ((m + qbezier2<fp>::p3().vec()) * sixit::units::create_dimensionless_scalar<fp>(fp_05));
			if (p0_22.sqr_magnitude() <= dd)
			{
				// p1 close to p2 - max curvature in p2
				auto p21_l = p21.magnitude();
				auto res = p21_l * p21_l * p21_l / area;
				return sixit::geometry::low_level::mathf::isfinite(res) ? res :
					(low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(p10, p21) > sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::square_meter>(fp_0)
						? sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_inf)
						: sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_0));
			}

			// common case
			auto p1_m = (qbezier2<fp>::p2().vec() - m).magnitude();
			return (area * area) / (p1_m * p1_m * p1_m);
		}

		template <typename fp>
		std::optional<qbezier2<fp>> qbezier2<fp>::build2(const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p0, 
															const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& p1, 
															const low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>& t0, 
															const low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>& t1)
		{
			low_level::dimensional_vector2 chord = p1 - p0;
			auto det = t0.x * t1.y - t0.y * t1.x;

			auto a = (chord.x * t1.y - chord.y * t1.x) / det;
			auto b = (t0.x * chord.y - t0.y * chord.x) / det;
			if (sixit::geometry::low_level::mathf::isfinite(a) && sixit::geometry::low_level::mathf::isfinite(b) && a > decltype(a)::zero() && b > decltype(b)::zero())
				return { qbezier2<fp>(point2<fp>(p0), point2<fp>(p0 + t0 * a), point2<fp>(p1)) };
			else
				return {};
		}

		template <typename fp>
		std::vector<qbezier2<fp>> qbezier2<fp>::elliptical_arc_to_qbeziers(const ellitical_arc_parameters<fp>& param)
		{
			auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f);
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);

			if (param.start == param.end)
				return {};


			auto pow2 = [](sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> val) {return val * val; };
			auto to_rad = [](fp deg) { return deg * fp(low_level::mathf::deg_2_rad); };
			auto rotate = [](point2<fp>& p, fp angle_rad) {
				auto X = p.vec().x * sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::cos(angle_rad)) - 
					   p.vec().y * sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::sin(angle_rad));
				
				p.vec().y = p.vec().x * sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::sin(angle_rad)) + 
							p.vec().y * sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::cos(angle_rad));
				p.vec().x = X; };

			const size_t num_of_segments = 6;
			const fp segment_angle = low_level::mathf::pi / num_of_segments;
			const fp L = fp(4.f) * low_level::mathf::tan(segment_angle / fp(4.f)) / fp(3.f);
			const fp angle = to_rad(low_level::mathf::fmod(sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(param.x_axis_rotation), fp(360.f)));

			std::vector<qbezier2<fp>> result;

			point2<fp> start = param.start, end = param.end;
			auto rx = low_level::mathf::abs(param.rx);
			auto ry = low_level::mathf::abs(param.ry);

			point2<fp> c;
			auto start_angle = sixit::units::create_dimensionless_scalar<fp>(0.f);
			auto end_angle = sixit::units::create_dimensionless_scalar<fp>(0.f);

			{
				rotate(start, -angle);
				rotate(end, -angle);

				point2<fp> d((start.vec().x - end.vec().x) / sixit::units::create_dimensionless_scalar<fp>(2.f), 
								(start.vec().y - end.vec().y) / sixit::units::create_dimensionless_scalar<fp>(2.f));

				auto h = pow2(d.vec().x) / pow2(rx) + pow2(d.vec().y) / pow2(ry);

				if (!sixit::geometry::low_level::mathf::isfinite(h))
					return {};

				if (h > fp_1) {
					h = low_level::mathf::sqrt(h);
					rx = h * rx;
					ry = h * ry;
				}

				auto left = pow2(rx) * pow2(ry) - pow2(rx) * pow2(d.vec().y) - pow2(ry) * pow2(d.vec().x);
				auto right = pow2(rx) * pow2(d.vec().y) + pow2(ry) * pow2(d.vec().x);
				auto k = sixit::units::create_dimensionless_scalar<fp>(param.large_arc_flag == param.sweep_flag ? -1.f : 1.f) * low_level::mathf::sqrt(low_level::mathf::abs(left / right));

				c.vec().set(k * rx * d.vec().y / ry + (start.vec().x + end.vec().x) / sixit::units::create_dimensionless_scalar<fp>(2.f), 
							k * -ry * d.vec().x / rx + (start.vec().y + end.vec().y) / sixit::units::create_dimensionless_scalar<fp>(2.f));

				start_angle = low_level::mathf::asin((start.vec().y - c.vec().y) / ry);
				end_angle = low_level::mathf::asin((end.vec().y - c.vec().y) / ry);

				if (start.vec().x < c.vec().x) {
					start_angle = sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::pi) - start_angle;
				}
				if (end.vec().x < c.vec().x) {
					end_angle = sixit::units::create_dimensionless_scalar<fp>(low_level::mathf::pi) - end_angle;
				}

				if (start_angle < decltype(start_angle)::zero()) {
					start_angle = sixit::units::create_dimensionless_scalar<fp>(fp(low_level::mathf::pi) * fp(2.f)) + start_angle;
				}
				if (end_angle < decltype(end_angle)::zero()) {
					end_angle = sixit::units::create_dimensionless_scalar<fp>(fp(low_level::mathf::pi) * fp(2.f)) + end_angle;
				}

				if (param.sweep_flag && start_angle > end_angle) {
					start_angle = start_angle - sixit::units::create_dimensionless_scalar<fp>(fp(low_level::mathf::pi) * fp(2.f));
				}
				if (!param.sweep_flag && end_angle > start_angle) {
					end_angle = end_angle - sixit::units::create_dimensionless_scalar<fp>(fp(low_level::mathf::pi) * fp(2.f));
				}
			}

			const auto hx = rx * sixit::units::create_dimensionless_scalar<fp>(L);
			const auto hy = ry * sixit::units::create_dimensionless_scalar<fp>(L);
			const auto sign = sixit::units::create_dimensionless_scalar<fp>((param.sweep_flag && (end_angle > start_angle)) ? fp(1.f) : fp(-1.f));

			auto get_path_point = [&](sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> el_angle) {
				auto x = c.vec().x + rx * low_level::mathf::cos(el_angle);
				auto y = c.vec().y + ry * low_level::mathf::sin(el_angle);
				point2 p1(x, y);
				point2 p2(x - sign * hx * low_level::mathf::sin(el_angle), y + sign * hy * low_level::mathf::cos(el_angle)); 
				rotate(p1, angle);
				rotate(p2, angle);
				return typename curve2<fp>::path_point(p1, direction2(p1, p2));
				};

			typename curve2<fp>::path_point pp_start = get_path_point(start_angle), pp_end;

			for (size_t t = 0; t < num_of_segments; ++t)
			{
				end_angle = start_angle + sign * sixit::units::create_dimensionless_scalar<fp>(segment_angle);
				pp_end = get_path_point(end_angle);
				auto qb = build2(pp_start.first.vec(), pp_end.first.vec(), pp_start.second.vec(), pp_end.second.vec());
				assert(qb.has_value()); 
				if (qb.has_value())
				{
					result.push_back(qb.value());
				}

				start_angle = end_angle;
				pp_start = pp_end;
			}

			return result;
		}


		template <typename fp>
		void qbezier2<fp>::intersection(std::vector<point2<fp>>& intersections, const typename curve2_primitive<fp>::pcurve_segment& other) const { return other->intersection(intersections, *this); }

		template <typename fp>
		void qbezier2<fp>::intersection(std::vector<point2<fp>>& intersections, const line2<fp>& other) const
		{
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);

			auto t = line_intersection_params(other.p1().vec(), other.p2().vec());
			if (!t.first.has_value())
				return;

			if (t.first.value() >= fp_0 && t.first.value() <= fp_1)
			{
				intersections.push_back(track_point(t.first.value()));
			}

			if (t.second.has_value() && t.second.value() >= fp_0 && t.second.value() <= fp_1)
			{
				intersections.push_back(track_point(t.second.value()));
			}
		}

		template <typename fp>
		void qbezier2<fp>::intersection(std::vector<point2<fp>>& intersections, const line_segment2<fp>& other) const
		{
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);

			if (other.p1() == other.p2()) 
				return;

			auto t = line_intersection_params(other.p1().vec(), other.p2().vec());
			if (!t.first.has_value())
				return;

			low_level::dimensional_vector2 line_dir = other.p2().vec() - other.p1().vec();
			auto sq_length = line_dir.sqr_magnitude();

			if (t.first.value() >= fp_0 && t.first.value() <= fp_1)
			{
				point2 ip = track_point(t.first.value());
				auto dp = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(ip.vec() - other.p1().vec(), line_dir) / sq_length;
				if (dp >= fp_0 && dp <= fp_1)
					intersections.push_back(ip);
			}

			// check second point
			if (t.second.has_value() && t.second.value() >= fp_0 && t.second.value() <= fp_1)
			{
				point2 ip2 = track_point(t.second.value());
				auto dp2 = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(ip2.vec() - other.p1().vec(), line_dir) / sq_length;
				if (dp2 >= fp_0 && dp2 <= fp_1)
					intersections.push_back(ip2);
			}
		}

		template <typename fp>
		void qbezier2<fp>::intersection(std::vector<point2<fp>>&, const arc2<fp>&) const {  }

		template <typename fp>
		void qbezier2<fp>::intersection(std::vector<point2<fp>>&, const qbezier2<fp>&) const {  }

		template <typename fp>
		void qbezier2<fp>::intersection(std::vector<point2<fp>>& intersections, const cbezier2<fp>& other) const { other.intersection(intersections, *this); }

		template <typename fp>
		bool qbezier2<fp>::intersect(const typename curve2_primitive<fp>::pcurve_segment& other) const { return other->intersect(*this); }

		template <typename fp>
		bool qbezier2<fp>::intersect(const line2<fp>& other) const 
		{ 
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);

			auto t = line_intersection_params(other.p1().vec(), other.p2().vec());
			if (!t.first.has_value())
				return false;

			if (t.first.value() >= fp_0 && t.first.value() <= fp_1)
				return true;

			if (t.second.has_value() && t.second.value() >= fp_0 && t.second.value() <= fp_1)
				return true;

			return false;
		}

		template <typename fp>
		bool qbezier2<fp>::intersect(const line_segment2<fp>& other) const 
		{
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);

			if (other.p1() == other.p2()) 
				return false;
			
			auto t = line_intersection_params(other.p1().vec(), other.p2().vec());
			if (!t.first.has_value())
				return false;

			low_level::dimensional_vector2 line_dir = other.p2().vec() - other.p1().vec();
			auto sq_length = line_dir.sqr_magnitude();

			if (t.first.value() >= fp_0 && t.first.value() <= fp_1)
			{
				point2 ip = track_point(t.first.value());
				auto dp = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(ip.vec() - other.p1().vec(), line_dir) / sq_length;
				if (dp >= fp_0 && dp <= fp_1)
					return true;
			}

				// check second point
			if (t.second.has_value() && t.second.value() >= fp_0 && t.second.value() <= fp_1)
			{
				point2 ip2 = track_point(t.second.value());
				auto dp2 = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(ip2.vec() - other.p1().vec(), line_dir) / sq_length;
				if (dp2 >= fp_0 && dp2 <= fp_1)
					return true;
			}
		
			return false; 
		}

		template <typename fp>
		bool qbezier2<fp>::intersect(const arc2<fp>&) const { return false; }

		template <typename fp>
		bool qbezier2<fp>::intersect(const qbezier2<fp>&) const { return false; }

		template <typename fp>
		bool qbezier2<fp>::intersect(const cbezier2<fp>& other) const { return other.intersect(*this); }

		template <typename fp>
		inline std::pair<typename std::optional<sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> >, std::optional<sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> >> 
			qbezier2<fp>::line_intersection_params(const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& lp1, 
													const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>& lp2) const
		{
			auto fp_2 = sixit::units::create_dimensionless_scalar<fp>(2.f);
			low_level::dimensional_vector2 l_dir = (lp2 - lp1).normalized();
			low_level::dimensional_vector2 ln(l_dir.y, -l_dir.x);

			low_level::dimensional_vector2 aa = qbezier2<fp>::p1().vec() - qbezier2<fp>::p2().vec() * fp_2 + qbezier2<fp>::p3().vec();
			low_level::dimensional_vector2 bb = (qbezier2<fp>::p2().vec() - qbezier2<fp>::p1().vec()) * fp_2;
			low_level::dimensional_vector2 cc = qbezier2<fp>::p1().vec() - lp1;

			auto a = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(aa, ln);
			auto b = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(bb, ln);
			auto c = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(cc, ln);

			return solve_quadric_equation(a, b, c);
		}

		template <typename fp>
		template<typename TSetPixel>
		inline void qbezier2<fp>::rasterize(TSetPixel& SET_PIXEL) const
		{			 
			//Alois Zingl method
			int x0 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p1().vec().x));
			int y0 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p1().vec().y));
			int x1 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p2().vec().x));
			int y1 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p2().vec().y));
			int x2 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p3().vec().x));
			int y2 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p3().vec().y));


			int x = x0 - x1, y = y0 - y1;
			fp t = sixit::guidelines::precision_cast<float>(x0 - 2 * x1 + x2);
			fp r;

			if (((x > 0) && (x2 > x1)) || ((x < 0) && (x2 < x1))) 
			{																// horizontal cut at P4? 
				if (((y > 0) && (y2 > y1)) || ((y < 0) && (y2 < y1)))		// vertical cut at P6 too? 
				{
					if (fp(float((y0 - 2 * y1 + y2) * x)) / t > fp(float(abs(y))))
					{														// which first? 
						x0 = x2; x2 = x + x1; y0 = y2; y2 = y + y1;			// swap points 
					}														// now horizontal cut at P4 comes first 
				}
				t = fp(float(x0 - x1)) / t;
				r = (fp(1.f) - t) * ((fp(1.f) - t) * fp(float(y0)) + fp(2.f) * t * fp(float(y1))) + t * t * fp(float(y2));	// By(t=P4) 
				t = fp(float(x0 * x2 - x1 * x1)) * t / fp(float(x0 - x1));					// gradient dP4/dx=0 
				x = low_level::mathf::fp2int(low_level::mathf::floor(t + fp(0.5f))); 
				y = low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f)));
				r = fp(float(y1 - y0)) * (t - fp(float(x0))) / fp(float(x1 - x0)) + fp(float(y0));					// intersect P3 | P0 P1 
				rasterize_qsegment(x0, y0, x, low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f))), x, y, SET_PIXEL);
				r = fp(float(y1 - y2)) * (t - fp(float(x2))) / fp(float(x1 - x2)) + fp(float(y2));					// intersect P4 | P1 P2 
				x0 = x1 = x; y0 = y; y1 = low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f)));					// P0 = P4, P1 = P8 
			}

			if (((y0 > y1) && (y2 > y1)) || ((y0 < y1) && (y2 < y1)))
			{																// vertical cut at P6? 
				t = sixit::guidelines::precision_cast<float>(y0 - 2 * y1 + y2);
				t = fp(float(y0 - y1)) / t;
				r = (fp(1.0f) - t) * ((fp(1.0f) - t) * fp(float(x0)) + fp(2.0f) * t * fp(float(x1))) + t * t * fp(float(x2));   // Bx(t=P6) 
				t = fp(float(y0 * y2 - y1 * y1)) * t / fp(float(y0 - y1));                    // gradient dP6/dy=0 
				x = low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f))); 
				y = low_level::mathf::fp2int(low_level::mathf::floor(t + fp(0.5f)));
				r = fp(float(x1 - x0)) * (t - fp(float(y0))) / fp(float(y1 - y0)) + fp(float(x0));                  // intersect P6 | P0 P1 
				rasterize_qsegment(x0, y0, low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f))), y, x, y, SET_PIXEL);
				r = fp(float(x1 - x2)) * (t - fp(float(y2))) / fp(float(y1 - y2)) + fp(float(x2));                  // intersect P7 | P1 P2 
				x0 = x; x1 = low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f))); y0 = y1 = y;					// P0 = P6, P1 = P7 
			}
			rasterize_qsegment(x0, y0, x1, y1, x2, y2, SET_PIXEL);          // remaining part 
		}

		template <typename fp>
		template<typename TSetPixel>
		inline void qbezier2<fp>::rasterize_qsegment(int x0, int y0, int x1, int y1, int x2, int y2, TSetPixel& SET_PIXEL) const
		{
			// plot a limited quadratic Bezier segment  (Alois Zingl method)
			int sx = x2 - x1, sy = y2 - y1;
			int xx = x0 - x1, yy = y0 - y1, xy;				// relative values for checks 
			fp cur = sixit::guidelines::precision_cast<float>(xx * sy - yy * sx);		// curvature 

			assert(xx * sx <= 0 && yy * sy <= 0);				// sign of gradient must not change 

			if (sx * sx + sy * sy > xx * xx + yy * yy) 
			{      // begin with longer part 
				x2 = x0; x0 = sx + x1; y2 = y0; y0 = sy + y1; cur = -cur; // swap P0 P2 
			}
			if (cur != fp(0))
            { // no straight line
                xx += sx;
                xx *= sx = x0 < x2 ? 1 : -1; // x step direction
                yy += sy;
                yy *= sy = y0 < y2 ? 1 : -1; // y step direction
                xy = 2 * xx * yy;
                xx *= xx;
                yy *= yy; // differences 2nd degree
                if (cur * fp(float(sx * sy)) < fp(0.f))
                { // negated curvature?
                    xx = -xx;
                    yy = -yy;
                    xy = -xy;
                    cur = -cur;
                }
                fp dx = fp(4.0f) * cur * fp(sixit::guidelines::precision_cast<float>(sy * (x1 - x0))) +
                        fp(sixit::guidelines::precision_cast<float>(xx - xy)); // differences 1st degree
                fp dy = fp(4.0f) * cur * fp(sixit::guidelines::precision_cast<float>(sx * (y0 - y1))) +
                        fp(sixit::guidelines::precision_cast<float>(yy - xy));
                xx += xx;
                yy += yy;
                fp err = dx + dy + fp(sixit::guidelines::precision_cast<float>(xy)); // error 1st step
                do
                {
                    SET_PIXEL(x0, y0); // plot curve
                    if (x0 == x2 && y0 == y2)
                        return;              // last pixel -> curve finished
                    y1 = fp(2.f) * err < dx; // save value for test of y step
                    if (fp(2.f) * err > dy)
                    {
                        x0 += sx;
                        dx = dx - fp(float(xy));
                        dy = dy + fp(float(yy));
                        err = err + dy;
                    } // x step
                    if (y1)
                    {
                        y0 += sy;
                        dy = dy - fp(float(xy));
                        dx = dx + fp(float(xx));
                        err = err + dx;
                    }                                   // y step
                } while (dy < fp(0.f) && dx > fp(0.f)); // gradient negates -> algorithm fails
            }

            fp fx0 = sixit::guidelines::precision_cast<float>(x0);
			fp fy0 = sixit::guidelines::precision_cast<float>(y0);
			fp fx2 = sixit::guidelines::precision_cast<float>(x2);
			fp fy2 = sixit::guidelines::precision_cast<float>(y2);
			line_segment2<fp>::rasterize(point2<fp>(
					sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fx0), 
					sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fy0)), 
				point2<fp>(
					sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fx2), 
					sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fy2)), SET_PIXEL); // plot remaining part to end 
		}

		template <typename fp>
		template <typename TSetPixelAA>
        inline void qbezier2<fp>::rasterize_aa(TSetPixelAA& SET_PIXEL_AA) const 
		{
			fp fp_1(1.f);
			fp fp_2(2.f);
			//Alois Zingl method
			int x0 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p1().vec().x));
			int y0 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p1().vec().y));
			int x1 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p2().vec().x));
			int y1 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p2().vec().y));
			int x2 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p3().vec().x));
			int y2 = low_level::mathf::fp2int(low_level::mathf::floor(qbezier2<fp>::p3().vec().y));

			int x = x0 - x1, y = y0 - y1;
			fp t = sixit::guidelines::precision_cast<float>(x0 - 2 * x1 + x2);
			fp r;

			if (((x > 0) && (x2 > x1)) || ((x < 0) && (x2 < x1))) 
			{																// horizontal cut at P4? 
				if (((y > 0) && (y2 > y1)) || ((y < 0) && (y2 < y1)))		// vertical cut at P6 too? 
				{
					if ( fp(float(abs(y0 - 2 * y1 + y2)) * x) / t > fp(float(abs(y))))
					{														// which first? 
						x0 = x2; x2 = x + x1; y0 = y2; y2 = y + y1;			// swap points 
					}														// now horizontal cut at P4 comes first 
				}
				t = fp(float(x0 - x1)) / t;
				r = (fp_1 - t) * ((fp_1 - t) * fp(float(y0)) + fp_2 * t * fp(float(y1))) + t * t * fp(float(y2));	// By(t=P4) 
				t = fp(float(x0 * x2 - x1 * x1)) * t / fp(float(x0 - x1));					// gradient dP4/dx=0 
				x = low_level::mathf::fp2int(low_level::mathf::floor(t + fp(0.5f))); 
				y = low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f)));
				r = fp(float(y1 - y0)) * (t - fp(float(x0))) / fp(float(x1 - x0)) + fp(float(y0));					// intersect P3 | P0 P1 
				rasterize_qsegment_aa(x0, y0, x, low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f))), x, y, SET_PIXEL_AA);
				r = fp(float(y1 - y2)) * (t - fp(float(x2))) / fp(float(x1 - x2)) + fp(float(y2));					// intersect P4 | P1 P2 
				x0 = x1 = x; y0 = y;
				y1 = low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f)));					// P0 = P4, P1 = P8 
			}

			if (((y0 > y1) && (y2 > y1)) || ((y0 < y1) && (y2 < y1)))
			{																// vertical cut at P6? 
				t = sixit::guidelines::precision_cast<float>(y0 - 2 * y1 + y2);
				t = fp(sixit::guidelines::precision_cast<float>(y0 - y1)) / t;
				r = (fp_1 - t) * ((fp_1 - t) * fp(float(x0)) + fp_2 * t * fp(float(x1))) + t * t * fp(float(x2));   // Bx(t=P6) 
				t = fp(float(y0 * y2 - y1 * y1)) * t / fp(float(y0 - y1));                    // gradient dP6/dy=0 
				x = low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f)));
				y = low_level::mathf::fp2int(low_level::mathf::floor(t + fp(0.5f)));
				r = fp(float(x1 - x0)) * (t - fp(float(y0))) / fp(float(y1 - y0)) + fp(float(x0));                  // intersect P6 | P0 P1 
				rasterize_qsegment_aa(x0, y0, low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f))), y, x, y, SET_PIXEL_AA);
				r = fp(float(x1 - x2)) * (t - fp(float(y2))) / fp(float(y1 - y2)) + fp(float(x2));                  // intersect P7 | P1 P2 
				x0 = x; x1 = low_level::mathf::fp2int(low_level::mathf::floor(r + fp(0.5f))); y0 = y1 = y;					// P0 = P6, P1 = P7 
			}
			rasterize_qsegment_aa(x0, y0, x1, y1, x2, y2, SET_PIXEL_AA);    // remaining part 
		}

		template <typename fp>
        template<typename TSetPixelAA>
        inline void qbezier2<fp>::rasterize_qsegment_aa(int x0, int y0, int x1, int y1, int x2, int y2, TSetPixelAA& SET_PIXEL_AA) const
		{
			// draw an limited anti-aliased quadric Bezier segment (Alois Zingol method)
			int sx = x2 - x1; 
			int sy = y2 - y1;
			int xx = x0 - x1;
			int yy = y0 - y1;
			int xy = (xx * yy) << 1;
			fp cur = sixit::guidelines::precision_cast<float>(xx * sy - yy * sx);								/* relative values for checks */
			assert(xx * sx <= 0 && yy * sy <= 0);						/* sign of gradient must not change */
			if (sx * sx + sy * sy > xx * xx + yy * yy) 		/* begin with longer part */
			{															/* swap P0 P2 */
				x2 = x0; 
				x0 = sx + x1; 
				y2 = y0; 
				y0 = sy + y1; 
				cur = -cur; 
			}
			if (cur != fp(0.f))
			{ /* no straight line */
				xx += sx;
				sx = x0 < x2 ? 1 : -1; 
				xx *= sx; 												/* x step direction */
				yy += sy; 
				sy = y0 < y2 ? 1 : -1;
				yy *= sy;							 					/* y step direction */
				xy = (xx * yy) << 1; 
				xx *= xx; 
				yy *= yy; 												/* differences 2nd degree */
				if (cur * fp(float(sx * sy)) < fp(0.f)) 
				{ 														/* negated curvature? */
					xx = -xx; 
					yy = -yy; 
					xy = -xy; 
					cur = -cur;
				}
				fp dx = fp(4.0f) * cur * fp(sixit::guidelines::precision_cast<float>(sy * (x1 - x0))) + fp(sixit::guidelines::precision_cast<float>(xx - xy)); 		/* differences 1st degree */
				fp dy = fp(4.0f) * cur * fp(sixit::guidelines::precision_cast<float>(sx * (y0 - y1))) + fp(sixit::guidelines::precision_cast<float>(yy - xy));
				xx += xx; 
				yy += yy; 
				fp err = dx + dy + fp(sixit::guidelines::precision_cast<float>(xy)); 								/* error 1st step */
				do {

					cur = dx + fp(float(xy));
					fp ed = fp(float(-xy)) - dy;
					if (cur > ed)
					{
						std::swap(cur, ed);
					}
					 													/* approximate error distance */
					fp ed1 = fp(1.f) / (ed + fp(2.f) * ed * cur * cur / (fp(4.f) * ed * ed + cur * cur));
					SET_PIXEL_AA(x0, y0, fp(1.f) - ed1 * low_level::mathf::abs(err - dx - dy - fp(float(xy))));
					if (x0 == x2 && y0 == y2)
					{
						return;											/* last pixel -> curve finished */
					}							
					x1 = x0; cur = dx-err; y1 = fp(2.f) * err+dy < fp(0.f);
					if (fp(2.f) * err+dx > fp(0.f)) 
					{ /* x step */
						if (err-dy < ed) 
						{
							SET_PIXEL_AA(x0, y0 + sy, fp(1.f) - ed1 * low_level::mathf::abs(err - dy));
						}
						x0 += sx; 
						dx = dx - fp(float(xy));
						dy = dy + fp(float(yy));
						err = err + dy;
					}
					if (y1) 
					{ /* y step */
						if (cur < ed)
						{
							SET_PIXEL_AA(x1 + sx,y0, fp(1.f) - ed1 * low_level::mathf::abs(cur));
						}
						y0 += sy; 
						dy = dy - fp(float(xy));
						dx = dx + fp(float(xx)); 
						err = err + dx;
					}
				} while (dy < dx); 										/* gradient negates -> close curves */
			}
			fp fx0 = sixit::guidelines::precision_cast<float>(x0);
			fp fy0 = sixit::guidelines::precision_cast<float>(y0);
			fp fx2 = sixit::guidelines::precision_cast<float>(x2);
			fp fy2 = sixit::guidelines::precision_cast<float>(y2);
			line_segment2<fp>::rasterize_aa(point2<fp>(
					sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fx0), 
					sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fy0)), 
				point2<fp>(
					sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fx2), 
					sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fy2)), SET_PIXEL_AA);
		}

		template <typename fp>
		template<typename TSetPixel>
		inline void cbezier2<fp>::rasterize(TSetPixel& SET_PIXEL) const
		{
			//divide into 2 qbeziers
			std::vector<typename curve2_primitive<fp>::pcurve_segment> qbezier_curves = to_qbezier(2);
			for (const auto& qb : qbezier_curves) 
			{
				switch (qb->get_primitive_type())
				{
				case curve2_primitive<fp>::primitive_type::line_segment:
					sixit::lwa::bit_cast<line_segment2<fp>*>(qb.get())->rasterize(SET_PIXEL);
					break;
				case curve2_primitive<fp>::primitive_type::qbezier:
					sixit::lwa::bit_cast<qbezier2<fp>*>(qb.get())->rasterize(SET_PIXEL);
					break;
				default:
					break;
				}
			}
		}

		template <typename fp>
		template<typename TSetPixelAA>
        inline void cbezier2<fp>::rasterize_aa(TSetPixelAA& SET_PIXEL_AA) const 
		{
			std::vector<typename curve2_primitive<fp>::pcurve_segment> qbezier_curves = to_qbezier(2);
			for (const auto& qb : qbezier_curves) 
			{
				switch (qb->get_primitive_type())
				{
				case curve2_primitive<fp>::primitive_type::line_segment:
					sixit::lwa::bit_cast<line_segment2<fp>*>(qb.get())->rasterize_aa(SET_PIXEL_AA);
					break;
				case curve2_primitive<fp>::primitive_type::qbezier:
					sixit::lwa::bit_cast<qbezier2<fp>*>(qb.get())->rasterize_aa(SET_PIXEL_AA);
					break;
				default:
					break;
				}
			}
		}
        
		template <typename fp>
		std::vector<typename curve2_primitive<fp>::pcurve_segment> cbezier2<fp>::to_qbezier(int count) const
		{
			typename std::vector<typename curve2_primitive<fp>::pcurve_segment> qbeziers;
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
			auto t0 = fp_0;
			auto p0 = track_point(t0);
			auto d0 = track_tangent(t0);
			auto step = fp_1 / sixit::units::create_dimensionless_scalar<fp>(fp(float(count)));
			for (int i = 0; i < count; i++)
			{
				auto t1 = t0 + step;
				auto p1 = track_point(t1);
				auto d1 = track_tangent(t1);
				auto qb = qbezier2<fp>::build2(p0.vec(), p1.vec(), d0.vec(), d1.vec());
				if (qb.has_value())
				{
					qbeziers.push_back(std::make_unique<qbezier2<fp>>(qb.value()));
				}
				else
				{
					qbeziers.push_back(std::make_unique<line_segment2<fp>>(line_segment2<fp>(p0, p1)));
				}

				t0 = t1;
				p0 = p1;
				d0 = d1;
			}
			return qbeziers;
		}

		// curve2 implementation
		template <typename fp>
		std::pair<point2<fp>, rotation2<fp>> curve2<fp>::track(class_value x) const
		{
			if (segments.empty())
				return { initial_point.first, rotation2<fp>(direction2<fp>(
					sixit::units::create_dimensionless_scalar<fp>(0.f), 
					sixit::units::create_dimensionless_scalar<fp>(1.f)), initial_point.second) };

			auto rest_length = x;
			for (const auto& it : segments)
			{
				auto l = it->length();
				if (l >= rest_length)
				{
					auto t = rest_length / l;
					return { it->track_point(t), rotation2<fp>(direction2<fp>(
						sixit::units::create_dimensionless_scalar<fp>(0.f), 
						sixit::units::create_dimensionless_scalar<fp>(1.f)), it->track_tangent(t)) };
				}
				else
					rest_length = rest_length - l;
			}

			return { end_point(), rotation2(direction2<fp>(
				sixit::units::create_dimensionless_scalar<fp>(0.f), 
				sixit::units::create_dimensionless_scalar<fp>(1.f)), end_tangent())};
		}
	
		template <typename fp>
		point2<fp> curve2<fp>::track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
		{
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);

			if (t <= fp_0)
				return start_point();

			if (t >= fp_1)
				return end_point();

			auto rest_length = t * length();
			for (const auto& it : segments)
			{
				auto l = it->length();
				if (l >= rest_length)
					return it->track_point(rest_length / l);
				else
					rest_length = rest_length - l;
			}

			return end_point();
		}

		template <typename fp>
		direction2<fp> curve2<fp>::track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const
		{
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);

			if (t <= fp_0)
				return start_tangent();

			if (t >= fp_1)
				return end_tangent();

			auto rest_length = t * length();
			for (const auto& it : segments)
			{
				auto l = it->length();
				if (l >= rest_length)
					return it->track_tangent(rest_length / l);
				else
					rest_length = rest_length - l;
			}

			return end_tangent();
		}

		template <typename fp>
		point2<fp> curve2<fp>::end_point() const 
		{
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
			return segments.empty() ? initial_point.first : segments.back()->track_point(fp_1);
		}

		template <typename fp>
		point2<fp> curve2<fp>::start_point() const 
		{
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
			return segments.empty() ? initial_point.first : segments.front()->track_point(fp_0);
		}

		template <typename fp>
		direction2<fp> curve2<fp>::end_tangent() const 
		{
			auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
			return segments.empty() ? initial_point.second : segments.back()->track_tangent(fp_1);
		}

		template <typename fp>
		direction2<fp> curve2<fp>::start_tangent() const 
		{
			auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
			return segments.empty() ? initial_point.second : segments.front()->track_tangent(fp_0);
		}

		template <typename fp>
		void curve2<fp>::update(const bounds2<fp>& b, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> l, 
								sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> r)
		{
			bound_box.expand_by(b.minn);
			bound_box.expand_by(b.maxx);

			curve_length = curve_length + l;
			min_curv_radius = low_level::mathf::min(min_curv_radius, r);
		}

		template <typename fp>
		void curve2<fp>::recalculate()
		{
			fp fp_0(0.f);
			if (segments.empty())
            {
                bound_box = bounds2(initial_point.first, initial_point.first);
                curve_length = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_0);
                min_curv_radius = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::infinity);
                return;
            }

			const auto& fs = segments.front();
			bound_box = fs->bounds();
			curve_length = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(fp_0);
			min_curv_radius = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::infinity);

			for (const auto& it : segments)
			{
				update(it->bounds(), it->length(), it->min_curvature_radius());
			}
		}

		template<typename fp>
		inline std::vector<typename curve2<fp>::raster_edge> curve2<fp>::create_edge_table(const std::vector<point2<fp>>& points)
		{
			std::vector<raster_edge> edge_table;
			for (size_t i = 0; i < points.size(); i++) 
			{
				const auto& curr = points[i].vec();
				const auto& next = points[(i + 1) % points.size()].vec();

				if ((next.y - curr.y) == sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f))
					continue;

				auto cy = low_level::mathf::floor(curr.y + sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.5f));
				auto ny = low_level::mathf::floor(next.y + sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.5f));

				raster_edge edge;
				if (curr.y < next.y)
				{
					edge.ymin = low_level::mathf::fp2int(low_level::mathf::trunc(cy));
					edge.x = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(curr.x);
				}
				else
				{
					edge.ymin = low_level::mathf::fp2int(low_level::mathf::trunc(ny));
					edge.x = sixit::units::for_export_only_extract_fp_from_dimensional_scalar<sixit::units::meter>(next.x);
				}
				edge.ymax = low_level::mathf::fp2int(low_level::mathf::trunc(low_level::mathf::max(cy, ny)));
				edge.slope = sixit::units::extract_dim_less_scalar((next.x - curr.x) / (next.y - curr.y));

 				edge_table.push_back(edge);
			}

			auto sort_by_y_and_x = [](const raster_edge& a, const raster_edge& b)
			{
				if (a.ymin != b.ymin)
					return a.ymin < b.ymin;
				if (a.ymax != b.ymax)
					return a.ymax < b.ymax;
				if (a.x != b.x)
					return a.x < b.x;

				return a.slope < b.slope;
			};
			std::sort(edge_table.begin(), edge_table.end(), sort_by_y_and_x);
			return edge_table;
		}

		template<typename fp>
		template<typename TSetPixel>
		inline void curve2<fp>::rasterize_scanline(const std::vector<point2<fp>>& points, TSetPixel& SET_PIXEL)
		{
			std::vector<raster_edge> et = create_edge_table(points);
			if (et.empty())
				return;
			std::vector<raster_edge> aet;

			const int y_min = et.front().ymin;
			const int y_max = et.back().ymax;
			
			int etidx = 0;
			bool needssort = false;

			for (int y = y_min; y <= y_max || !aet.empty(); ++y)
			{
				//add newly found edges
				needssort = etidx < et.size() && et[etidx].ymin == y;
				while (etidx < et.size() && et[etidx].ymin == y)
					aet.push_back(et[etidx++]);
			
				for (size_t i = 1; i < aet.size() && !needssort; ++i) // TODO: checks only special cases 
				{
					needssort |= aet[i].x < aet[i - 1].x;
				}
				
				//sort by x intersection 
				if (needssort) 
				{
					std::sort(aet.begin(), aet.end(), [](const raster_edge& a, const raster_edge& b) 
						{ 
							if (a.x != b.x)
								return a.x < b.x;
							return a.slope < b.slope; 
						});
				}

				//set pixels
				bool needserase = false;
				for (size_t i = 0; i < aet.size();)
				{
					const auto& edge1 = aet[i];
					const auto& edge2 = aet[(i + 1) % aet.size()];
					int x1 = low_level::mathf::fp2int(low_level::mathf::floor(edge1.x + fp(0.5f)));
					int x2 = low_level::mathf::fp2int(low_level::mathf::floor(edge2.x));
					needserase |= (edge1.ymax <= y + 1 || edge2.ymax <= y + 1);
					if (x1 > x2)
					{
						std::swap(x1, x2);
					}
					for (int x = x1; x <= x2; x++)
					{
						SET_PIXEL(x, y);
					}
					if (x1 == x2 && (edge1.ymin == edge2.ymax || edge1.ymax == edge2.ymin))
					{
						++i;	//continuous edge
					}
					else
					{
						i += 2;
					}
				}

				//remove completed edges
				if (needserase)
				{
					aet.erase(std::remove_if(aet.begin(), aet.end(), [y](const raster_edge& edge) { return y+1 >= edge.ymax; }), aet.end());
				}

				//Update each edge's x value in the AET for the next scanline.
				for (raster_edge& edge : aet)
					edge.x = edge.x + edge.slope;
			}
		}

		template <typename fp>
		bool curve2<fp>::append(typename curve2_primitive<fp>::pcurve_segment&& p)
		{
			update(p->bounds(), p->length(), p->min_curvature_radius());
			segments.push_back(std::move(p));
			return true;
		}

		template <typename fp>
		void curve2<fp>::append(std::vector<typename curve2_primitive<fp>::pcurve_segment>& primitives)
		{
			for (auto it = std::make_move_iterator(primitives.begin()); it != std::make_move_iterator(primitives.end()); it++)
			{
				append(*it);
			}
		}

		template <typename fp>
		bool curve2<fp>::prepend(typename curve2_primitive<fp>::pcurve_segment&& p)
		{
			update(p->bounds(), p->length(), p->min_curvature_radius());
			segments.insert(segments.begin(), std::move(p));
			return true;
		}

		template <typename fp>
		void curve2<fp>::prepend(std::vector<typename curve2_primitive<fp>::pcurve_segment>& primitives)
		{
			for (auto it = std::make_move_iterator(primitives.rend()); it != std::make_move_iterator(primitives.rbegin()); it--)
			{
				prepend(*it);
			}
		}

		// one-primitive construct methods
		template <typename fp>
		bool curve2<fp>::append(const shape_line2<fp>&, const point2<fp>& dest_point)
		{
			return append(std::make_unique<line_segment2<fp>>(line_segment2<fp>(end_point(), dest_point)));
		}

		template <typename fp>
		bool curve2<fp>::append(const shape_arc2<fp>& arc, const point2<fp>& dest_point)
		{
			fp fp_05(0.5f);
			fp fp_1(1.f);
			const low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> end_vec = end_point().vec();
			low_level::dimensional_vector2 half_chord = (dest_point.vec() - end_vec) * sixit::units::create_dimensionless_scalar<fp>(fp_05);

			auto sq_radius = arc.radius * arc.radius;
			auto sq_half_chord = half_chord.sqr_magnitude();

			if (sq_half_chord > sq_radius)
				return false;

			auto sq_div = sq_radius / sq_half_chord; 
			if (!sixit::geometry::low_level::mathf::isfinite(sq_div))
				return false;
			
			auto coef = low_level::mathf::sqrt(sq_div - sixit::units::create_dimensionless_scalar<fp>(fp_1));
			low_level::dimensional_vector2 mid_point = (dest_point.vec() + end_vec) * sixit::units::create_dimensionless_scalar<fp>(fp_05);
			low_level::dimensional_vector2 r_norm(half_chord.y * coef, -half_chord.x * coef);
			low_level::dimensional_vector2 center = arc.cw ? mid_point + r_norm : mid_point - r_norm;

			low_level::dimensional_vector2 radius_vec0 = (end_vec - center).normalized();
			low_level::dimensional_vector2 radius_vec1 = (dest_point.vec() - center).normalized();

			auto angle0 = arc2<fp>::angle_from_radius_vector(radius_vec0);
			auto angle1 = arc2<fp>::norm_angle(angle0, arc2<fp>::angle_from_radius_vector(radius_vec1), arc.cw);

			return append(std::make_unique<arc2<fp>>(arc2<fp>(point2(center), arc.radius, angle0, angle1)));
		}

		template <typename fp>
		bool curve2<fp>::append_smooth(const shape_line2<fp>&, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length)
		{
			auto ep = end_point();
			return append(std::make_unique<line_segment2<fp>>(line_segment2<fp>(ep, ep.vec() + curve2<fp>::end_tangent().vec() * length)));
		}

		template <typename fp>
		bool curve2<fp>::append_smooth(const shape_arc2<fp>& arc, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length)
		{
			low_level::dimensional_vector2 r_vec = arc2<fp>::radius_vector_from_tangent(end_tangent().vec(), arc.cw);
			auto from_angle = arc2<fp>::angle_from_radius_vector(r_vec);
			auto delta_angle = length / arc.radius;
			auto to_angle = arc.cw ? (from_angle - delta_angle) : (from_angle + delta_angle);
			low_level::dimensional_vector2 center_vec = end_point().vec() - (r_vec * arc.radius);

			return append(std::make_unique<arc2<fp>>(arc2<fp>(point2(center_vec), arc.radius, from_angle, to_angle)));
		}

		template <typename fp>
		bool curve2<fp>::append_smooth(const shape_arc2<fp>& arc, const direction2<fp>& dest_dir)
		{
			low_level::dimensional_vector2 r_vec = arc2<fp>::radius_vector_from_tangent(end_tangent().vec(), arc.cw);
			auto from_angle = arc2<fp>::angle_from_radius_vector(r_vec);
			auto to_angle = arc2<fp>::angle_from_radius_vector(arc2<fp>::radius_vector_from_tangent(dest_dir.vec(), arc.cw));
			to_angle = arc2<fp>::norm_angle(from_angle, to_angle, arc.cw);
			low_level::dimensional_vector2 center_vec = end_point().vec() - (r_vec * arc.radius);

			return append(std::make_unique<arc2<fp>>(arc2<fp>(point2(center_vec), arc.radius, from_angle, to_angle)));
		}

		template <typename fp>
		bool curve2<fp>::append_smooth(const shape_arc2_w_radius<fp>& arc, const point2<fp>& dest_point)
		{
			fp fp_05(0.5f);
			low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> end_vec = end_point().vec();
			low_level::dimensional_vector2 half_chord = (dest_point.vec() - end_vec) * sixit::units::create_dimensionless_scalar<fp>(fp_05);

			low_level::dimensional_vector2 r_vec_from = arc2<fp>::radius_vector_from_tangent(end_tangent().vec(), arc.cw);
			auto hc_dot_rvec = -low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(half_chord, r_vec_from);
			if (hc_dot_rvec <= decltype(hc_dot_rvec)::zero())
				return false;

			auto radius = half_chord.sqr_magnitude() / hc_dot_rvec;
			if (!sixit::geometry::low_level::mathf::isfinite(radius))
				return false;

			low_level::dimensional_vector2 center = end_vec - r_vec_from * radius;
			low_level::dimensional_vector2 rad_vec_to = dest_point.vec() - center;

			auto from_angle = arc2<fp>::angle_from_radius_vector(r_vec_from);
			auto to_angle = arc2<fp>::norm_angle(from_angle, arc2<fp>::angle_from_radius_vector(direction2<fp>(rad_vec_to).vec()), arc.cw);
			return append(std::make_unique<arc2<fp>>(arc2<fp>(point2<fp>(center), radius, from_angle, to_angle)));
		}

		template <typename fp>
		bool curve2<fp>::append_smooth(const shape_qbezier2<fp>&, const path_point& dest_point)
		{
			auto qb = qbezier2<fp>::build2(end_point().vec(), dest_point.first.vec(), end_tangent().vec(), dest_point.second.vec());
			if (qb.has_value())
				return append(std::make_unique<qbezier2<fp>>(qb.value()));
			else
				return false;
		}

		template <typename fp>
		bool curve2<fp>::prepend(const shape_line2<fp>&, const point2<fp>& dest_point)
		{
			return prepend(std::make_unique<line_segment2<fp>>(line_segment2<fp>(dest_point, start_point())));
		}

		template <typename fp>
		bool curve2<fp>::prepend(const shape_arc2<fp>& arc, const point2<fp>& dest_point)
		{
			fp fp_05(0.5f);
			fp fp_1(1.f);

			low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> start_vec = start_point().vec();
			low_level::dimensional_vector2 half_chord = (start_vec - dest_point.vec()) * sixit::units::create_dimensionless_scalar<fp>(fp_05);

			auto sq_radius = arc.radius * arc.radius;
			auto sq_half_chord = half_chord.sqr_magnitude();

			if (sq_half_chord > sq_radius)
				return false;

			auto sq_div = sq_radius / sq_half_chord; 
			if (!sixit::geometry::low_level::mathf::isfinite(sq_div))
				return false;

			auto coef = low_level::mathf::sqrt(sq_div - sixit::units::create_dimensionless_scalar<fp>(fp_1));
			low_level::dimensional_vector2 mid_point = (dest_point.vec() + start_vec) * sixit::units::create_dimensionless_scalar<fp>(fp_05);
			low_level::dimensional_vector2 r_norm(half_chord.y * coef, -half_chord.x * coef);
			low_level::dimensional_vector2 center = arc.cw ? mid_point + r_norm : mid_point - r_norm;

			low_level::dimensional_vector2 radius_vec0 = (dest_point.vec() - center).normalized();
			low_level::dimensional_vector2 radius_vec1 = (start_vec - center).normalized();

			auto angle0 = arc2<fp>::angle_from_radius_vector(radius_vec0);
			auto angle1 = arc2<fp>::norm_angle(angle0, arc2<fp>::angle_from_radius_vector(radius_vec1), arc.cw);

			return prepend(std::make_unique<arc2<fp>>(arc2<fp>(point2(center), arc.radius, angle0, angle1)));
		}

		template <typename fp>
		bool curve2<fp>::prepend_smooth(const shape_line2<fp>&, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length)
		{
			auto sp = start_point();
			return prepend(std::make_unique<line_segment2<fp>>(line_segment2<fp>(sp.vec() - curve2<fp>::start_tangent().vec() * length, sp)));
		}

		template <typename fp>
		bool curve2<fp>::prepend_smooth(const shape_arc2<fp>& arc, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> length)
		{
			low_level::dimensional_vector2 r_vec = arc2<fp>::radius_vector_from_tangent(start_tangent().vec(), arc.cw);
			auto to_angle = arc2<fp>::angle_from_radius_vector(r_vec);
			auto delta_angle = length / arc.radius;
			auto from_angle = arc.cw ? (to_angle + delta_angle) : (to_angle - delta_angle);
			low_level::dimensional_vector2 center_vec = start_point().vec() - (r_vec * arc.radius);

			return prepend(std::make_unique<arc2<fp>>(arc2<fp>(point2(center_vec), arc.radius, from_angle, to_angle)));
		}

		template <typename fp>
		bool curve2<fp>::prepend_smooth(const shape_arc2<fp>& arc, const direction2<fp>& dest_dir)
		{
			low_level::dimensional_vector2 r_vec = arc2<fp>::radius_vector_from_tangent(start_tangent().vec(), arc.cw);
			auto to_angle = arc2<fp>::angle_from_radius_vector(r_vec);
			auto from_angle = arc2<fp>::angle_from_radius_vector(arc2<fp>::radius_vector_from_tangent(dest_dir.vec(), arc.cw));
			to_angle = arc2<fp>::norm_angle(from_angle, to_angle, arc.cw);
			low_level::dimensional_vector2 center_vec = start_point().vec() - (r_vec * arc.radius);

			return prepend(std::make_unique<arc2<fp>>(arc2<fp>(point2(center_vec), arc.radius, from_angle, to_angle)));
		}

		template <typename fp>
		bool curve2<fp>::prepend_smooth(const shape_arc2_w_radius<fp>& arc, const point2<fp>& dest_point)
		{
			fp fp_05(0.5f);

			low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> start_vec = start_point().vec();
			low_level::dimensional_vector2 half_chord = (start_vec - dest_point.vec()) * sixit::units::create_dimensionless_scalar<fp>(fp_05);

			low_level::dimensional_vector2 r_vec_to = arc2<fp>::radius_vector_from_tangent(start_tangent().vec(), arc.cw);
			auto hc_dot_rvec = low_level::dimensional_vector2<fp, sixit::units::length_unit::dim>::dot(half_chord, r_vec_to);
			if (hc_dot_rvec <= decltype(hc_dot_rvec)::zero())
				return false;

			auto radius = half_chord.sqr_magnitude() / hc_dot_rvec;
			if (!sixit::geometry::low_level::mathf::isfinite(radius))
				return false;

			low_level::dimensional_vector2 center = start_vec - r_vec_to * radius;
			low_level::dimensional_vector2 rad_vec_from = dest_point.vec() - center;

			auto from_angle = arc2<fp>::angle_from_radius_vector(direction2<fp>(rad_vec_from).vec());
			auto to_angle = arc2<fp>::norm_angle(from_angle, arc2<fp>::angle_from_radius_vector(direction2<fp>(r_vec_to).vec()), arc.cw);

			return prepend(std::make_unique<arc2<fp>>(arc2<fp>(point2(center), radius, from_angle, to_angle)));
		}

		template <typename fp>
		bool curve2<fp>::prepend_smooth(const shape_qbezier2<fp>&, const path_point& dest_point)
		{
			auto qb = qbezier2<fp>::build2(dest_point.first.vec(), start_point().vec(), dest_point.second.vec(), start_tangent().vec());
			if (qb.has_value())
				return prepend(std::make_unique<qbezier2<fp>>(qb.value()));
			else
				return false;
		}

		// multi-primitive construct methods
		template <typename fp>
		bool curve2<fp>::append_smooth(const shape_line_arc_line2<fp>& composite_shape, const curve2<fp>::path_point& dest_point)
		{
			path_point src_point({ end_point(), end_tangent() });
			auto composite = curv_builder<fp>::build(composite_shape, src_point, dest_point);
			append(composite);
			return !composite.empty();
		}

		template <typename fp>
		void curve2<fp>::append_smooth(const shape_arc_line_arc2<fp>& composite_shape, const curve2<fp>::path_point& dest_point)
		{
			path_point src_point({ end_point(), end_tangent() });
			auto composite = curv_builder<fp>::build(composite_shape, src_point, dest_point);
			append(composite);
		}

		template <typename fp>
		void curve2<fp>::append_smooth(const shape_composite_cbezier2<fp>& composite_shape, const curve2<fp>::path_point& dest_point)
		{
			path_point src_point({ end_point(), end_tangent() });
			auto composite = curv_builder<fp>::build(composite_shape, src_point, dest_point);
			append(composite);
		}

		template <typename fp>
		bool curve2<fp>::prepend_smooth(const shape_line_arc_line2<fp>& composite_shape, const curve2<fp>::path_point& dest_point)
		{
			path_point src_point({ end_point(), end_tangent() });
			auto composite = curv_builder<fp>::build(composite_shape, src_point, dest_point);
			prepend(composite);
			return !composite.empty();
		}

		template <typename fp>
		void curve2<fp>::prepend_smooth(const shape_arc_line_arc2<fp>& composite_shape, const curve2<fp>::path_point& dest_point)
		{
			path_point src_point({ end_point(), end_tangent() });
			auto composite = curv_builder<fp>::build(composite_shape, src_point, dest_point);
			prepend(composite);
		}

		template <typename fp>
		void curve2<fp>::prepend_smooth(const shape_composite_cbezier2<fp>& composite_shape, const curve2<fp>::path_point& dest_point) 
		{
			path_point src_point({ end_point(), end_tangent() });
			auto composite = curv_builder<fp>::build(composite_shape, src_point, dest_point);
			prepend(composite);
		}

		// public curve intersections methods
		template <typename fp>
		std::vector<point2<fp>> curve2<fp>::intersection(const line2<fp>& line) const
		{
			std::vector<point2<fp>> intersections = std::vector<point2<fp>>();
			intersection(intersections, line);
			return intersections;
		}

		template <typename fp>
		std::vector<point2<fp>> curve2<fp>::intersection(const line_segment2<fp>& line_segment) const
		{
			std::vector<point2<fp>> intersections = std::vector<point2<fp>>();
			intersection(intersections, line_segment);
			return intersections;
		}

		template <typename fp>
		std::vector<point2<fp>> curve2<fp>::intersection(const curve2& other) const
		{
			std::vector<point2<fp>> intersections = std::vector<point2<fp>>();
			intersection(intersections, other);
			return intersections;
		}

		template <typename fp>
		void curve2<fp>::intersection(std::vector<point2<fp>>& intersections, const typename curve2_primitive<fp>::pcurve_segment& other) const
		{
			other->intersection(intersections, *this);
		}

		template <typename fp>
		void curve2<fp>::intersection(std::vector<point2<fp>>& intersections, const line2<fp>& other) const
		{
			for (const auto& it : segments)
			{
				it->intersection(intersections, other);
			}
		}

		template <typename fp>
		void curve2<fp>::intersection(std::vector<point2<fp>>& intersections, const line_segment2<fp>& other) const
		{
			for (const auto& it : segments)
			{
				it->intersection(intersections, other);
			}
		}

		template <typename fp>
		void curve2<fp>::intersection(std::vector<point2<fp>>& intersections, const arc2<fp>& other) const
		{
			for (const auto& it : segments)
			{
				it->intersection(intersections, other);
			}
		}

		template <typename fp>
		void curve2<fp>::intersection(std::vector<point2<fp>>& intersections, const qbezier2<fp>& other) const
		{
			for (const auto& it : segments)
			{
				it->intersection(intersections, other);
			}
		}

		template <typename fp>
		void curve2<fp>::intersection(std::vector<point2<fp>>& intersections, const cbezier2<fp>& other) const
		{
			for (const auto& it : segments)
			{
				it->intersection(intersections, other);
			}
		}

		template <typename fp>
		void curve2<fp>::intersection(std::vector<point2<fp>>& intersections, const curve2<fp>& other) const
		{
			for (const auto& it_a : segments)
			{
				for (const auto& it_b : other.segments)
				{
					it_a->intersection(intersections, it_b);
				}
			}
		}

		template <typename fp>
		bool curve2<fp>::intersect(const typename curve2_primitive<fp>::pcurve_segment& other) const
		{
			for (const auto& it : segments)
			{
				if (it->intersect(other))
					return true;
			}
			return false;
		}

		template <typename fp>
		bool curve2<fp>::intersect(const line2<fp>& line) const
		{
			for (const auto& it : segments)
			{
				if (it->intersect(line))
					return true;
			}
			return false;
		}

		template <typename fp>
		bool curve2<fp>::intersect(const line_segment2<fp>& line_segment) const
		{
			for (const auto& it : segments)
			{
				if (it->intersect(line_segment))
					return true;
			}
			return false;
		}

		template <typename fp>
		bool curve2<fp>::intersect(const arc2<fp>& other) const
		{
			for (const auto& it : segments)
			{
				if (it->intersect(other))
					return true;
			}
			return false;
		}

		template <typename fp>
		bool curve2<fp>::intersect(const qbezier2<fp>& other) const
		{
			for (const auto& it : segments)
			{
				if (it->intersect(other))
					return true;
			}
			return false;
		}

		template <typename fp>
		bool curve2<fp>::intersect(const cbezier2<fp>& other) const
		{
			for (const auto& it : segments)
			{
				if (it->intersect(other))
					return true;
			}
			return false;
		}

		template <typename fp>
		bool curve2<fp>::intersect(const curve2& other) const
		{
			for (const auto& it_a : segments)
			{
				for (const auto& it_b : other.segments)
				{
					if (it_a->intersect(it_b))
						return true;
				}
			}
			return false;
		}

		template <typename fp>
		void curve2<fp>::to_quadbeziers()
		{
			std::vector<typename curve2_primitive<fp>::pcurve_segment> clone;
			for (auto it = std::make_move_iterator(segments.begin()); it != std::make_move_iterator(segments.end()); it++)
			{
				if ((*it)->get_primitive_type() == curve2_primitive<fp>::primitive_type::cbezier)
				{
					const cbezier2<fp>& cbezier_segment = dynamic_cast<const cbezier2<fp>&>(*(*it));
					auto qbeziers = cbezier_segment.to_qbezier(4);

					for (auto b_it = std::make_move_iterator(qbeziers.begin()); b_it != std::make_move_iterator(qbeziers.end()); b_it++)
					{
						clone.push_back(std::move(*b_it));
					}
				}
				else
					clone.push_back(std::move(*it));
			}

			segments.clear();
			segments = std::move(clone);

			recalculate();
		}

		template<typename fp>
		template<typename TSetPixel>
		inline void curve2<fp>::rasterize(TSetPixel& SET_PIXEL, curve2<fp>::class_value thickness) const
		{
			fp dist = 20.f;
			if (!segments.empty())
			{
				if (thickness <= 1)
				{
					for (const auto& segment : segments)
						raster_segment(segment, SET_PIXEL, dist);
				}
				else
				{
					auto it_prev = segments.begin();
					for (auto it = segments.begin(); it != segments.end(); ++it)
					{
						raster_segment(*it, SET_PIXEL, dist, thickness);
						
						//build edge connection if needed:
						raster_corner(*it_prev, *it, SET_PIXEL, thickness);
						it_prev = it;
					}
				}
			}
		}

		template <typename fp>
		template<typename TSetPixel>
		inline void curve2<fp>::raster_segment(const typename curve2_primitive<fp>::pcurve_segment& segment, TSetPixel& SET_PIXEL, 
												sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> dist_between_points) const
		{
			switch (segment->get_primitive_type())
			{
			case curve2_primitive<fp>::primitive_type::line_segment:
				sixit::lwa::bit_cast<line_segment2<fp>*> (segment.get())->rasterize(SET_PIXEL);
				break;
			case curve2_primitive<fp>::primitive_type::qbezier:
				sixit::lwa::bit_cast<qbezier2<fp>*>(segment.get())->rasterize(SET_PIXEL);
				break;
			case curve2_primitive<fp>::primitive_type::cbezier:
				sixit::lwa::bit_cast<cbezier2<fp>*>(segment.get())->rasterize(SET_PIXEL);
				break;
			case curve2_primitive<fp>::primitive_type::arc:
				sixit::lwa::bit_cast<arc2<fp>*>(segment.get())->rasterize(SET_PIXEL, dist_between_points);
				break;
			default: break;
			}			
		}

		template <typename fp>
		template <typename TSetPixel>
		inline void curve2<fp>::raster_segment(const typename curve2_primitive<fp>::pcurve_segment& segment, TSetPixel& SET_PIXEL, 
												sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> dist_between_points, 
												sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness) const
		{
			std::vector<std::pair<point2<fp>, rotation2<fp>>> segment_points;
			const auto segment_len = segment->length();
			switch (segment->get_primitive_type())
			{
			case curve2_primitive<fp>::primitive_type::line_segment:
				segment_points.push_back(segment->track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero()));
				segment_points.push_back(segment->track(segment_len));
				break;

			case curve2_primitive<fp>::primitive_type::arc:
			case curve2_primitive<fp>::primitive_type::qbezier:
			case curve2_primitive<fp>::primitive_type::cbezier:
			{
				const auto point_count = low_level::mathf::max(sixit::units::create_dimensionless_scalar<fp>(4.f), low_level::mathf::floor(segment_len / dist_between_points));
				auto delta = segment_len / point_count;

				for (auto delta_acc = decltype(delta)::zero(); delta_acc < segment_len; delta_acc += delta)
					segment_points.push_back(segment->track(delta_acc));
			}
			break;
			default: break;
			}

			//create a polygon frame around the line, then raster
			std::vector<point2<fp>> left;
			std::vector<point2<fp>> right;
			for (const auto& pair_point_rotation : segment_points)
			{
				auto& pos = pair_point_rotation.first.vec();
				const rotation2<fp>& rot = pair_point_rotation.second;

				low_level::dimensional_vector2 perpendicular = rot.rotate(direction2<fp>::right()).vec() * thickness * sixit::units::create_dimensionless_scalar<fp>(0.5f);
				left.push_back(pos - perpendicular);
				right.push_back(pos + perpendicular);
			}
			left.insert(left.end(), right.rbegin(), right.rend()); //reverse and append

			rasterize_scanline(left, SET_PIXEL);
		}

		template<typename fp>
		template<typename TSetPixel>
		void curve2<fp>::raster_corner(const typename curve2_primitive<fp>::pcurve_segment& segment1, const typename curve2_primitive<fp>::pcurve_segment& segment2, 
											TSetPixel& SET_PIXEL, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness)
		{
			auto path1 = segment1->track(segment1->length());
			auto path2 = segment2->track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero());

			auto p1 = path1.first.vec();
			auto p2 = path2.first.vec();
			direction2 dir1 = path1.second.rotate(direction2<fp>::down());
			direction2 dir2 = path2.second.rotate(direction2<fp>::down());
			thickness = thickness * sixit::units::create_dimensionless_scalar<fp>(0.5f);
			low_level::dimensional_vector2 perp1 = path1.second.rotate(direction2<fp>::right()).vec() * thickness;
			low_level::dimensional_vector2 perp2 = path2.second.rotate(direction2<fp>::right()).vec() * thickness;

			low_level::dimensional_vector2 e1 = p1 - perp1;
			low_level::dimensional_vector2 e2 = p2 - perp2;
			low_level::dimensional_vector2 e3 = p2 + perp2;
			low_level::dimensional_vector2 e4 = p1 + perp1;

			low_level::dimensional_vector2 v1 = e2 - e1;
			low_level::dimensional_vector2 v2 = e4 - e3;

			if (v1.x * v2.y - v2.x * v1.y < sixit::units::dimensional_scalar<fp, sixit::units::area_unit::dim>::zero())
				rasterize_scanline({ e1, e2, e3, e4 }, SET_PIXEL);
			else
				rasterize_scanline({ e1, e2, e4, e3 }, SET_PIXEL);
		}

		template<typename fp>
		template<typename TSetPixelAA>
		inline void curve2<fp>::rasterize_aa(TSetPixelAA& SET_PIXEL_AA, curve2<fp>::class_value thickness) const 
		{
			fp dist = 20.f;
			if (!segments.empty())
			{
				if (thickness <= 1)
				{
					for (const auto& segment : segments)
						raster_segment_aa(segment, SET_PIXEL_AA, dist);
				}
				else
				{
					auto it_prev = segments.begin();
					auto set_pixel = [&](int x, int y) { SET_PIXEL_AA(x, y, 1.f); };
					for (auto it = segments.begin(); it != segments.end(); ++it)
					{
						raster_segment_aa(*it, SET_PIXEL_AA, dist, thickness);
						
						//build edge connection if needed:
						raster_corner(*it_prev, *it, set_pixel, thickness);
						it_prev = it;
					}
				}
			}
		}

		template<typename fp>
		template<typename TSetPixelAA>
		inline void curve2<fp>::raster_segment_aa(const typename curve2_primitive<fp>::pcurve_segment& segment, TSetPixelAA& SET_PIXEL_AA, 
													sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> dist_between_points) const
		{
			switch (segment->get_primitive_type())
			{
			case curve2_primitive<fp>::primitive_type::line_segment:
				sixit::lwa::bit_cast<line_segment2<fp>*>(segment.get())->rasterize_aa(SET_PIXEL_AA);
				break;
			case curve2_primitive<fp>::primitive_type::qbezier:
				sixit::lwa::bit_cast<qbezier2<fp>*>(segment.get())->rasterize_aa(SET_PIXEL_AA);
				break;
			case curve2_primitive<fp>::primitive_type::cbezier:
				sixit::lwa::bit_cast<cbezier2<fp>*>(segment.get())->rasterize_aa(SET_PIXEL_AA);
				break;
			case curve2_primitive<fp>::primitive_type::arc:
				sixit::lwa::bit_cast<arc2<fp>*>(segment.get())->rasterize_aa(SET_PIXEL_AA, dist_between_points);
				break;
			default: break;
			}	
		}

		template<typename fp>
		template<typename TSetPixelAA>
		inline void curve2<fp>::raster_segment_aa(const typename curve2_primitive<fp>::pcurve_segment& segment, TSetPixelAA& SET_PIXEL_AA, 
													sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> dist_between_points, 
													sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> thickness) const
		{
			std::vector<std::pair<point2<fp>, rotation2<fp>>> segment_points;
			const auto segment_len = segment->length();
			switch (segment->get_primitive_type())
			{
			case curve2_primitive<fp>::primitive_type::line_segment:
				segment_points.push_back(segment->track(sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>::zero()));
				segment_points.push_back(segment->track(segment_len));
				break;

			case curve2_primitive<fp>::primitive_type::arc:
			case curve2_primitive<fp>::primitive_type::qbezier:
			case curve2_primitive<fp>::primitive_type::cbezier:
			{
				const auto point_count = low_level::mathf::max(sixit::units::create_dimensionless_scalar<fp>(4.f), low_level::mathf::floor(segment_len / dist_between_points));
				auto delta = segment_len / point_count;

				for (auto delta_acc = decltype(delta)::zero(); delta_acc <= segment_len; delta_acc += delta)
					segment_points.push_back(segment->track(delta_acc));
			}
			break;
			default: break;
			}

			//create a polygon frame around the line, then raster
			std::vector<point2<fp>> left;
			std::vector<point2<fp>> right;
			for (const auto& pair_point_rotation : segment_points)
			{
				auto& pos = pair_point_rotation.first.vec();
				const rotation2<fp>& rot = pair_point_rotation.second;

				low_level::dimensional_vector2 perpendicular = rot.rotate(direction2<fp>::right()).vec() * thickness * sixit::units::create_dimensionless_scalar<fp>(0.5f);
				left.push_back(pos - perpendicular);
				right.push_back(pos + perpendicular);
			}
			left.insert(left.end(), right.rbegin(), right.rend()); //reverse and append
			line_segment2<fp>::rasterize_aa(*left.begin(), *right.begin(), SET_PIXEL_AA);

			for (size_t i = 0; i < left.size() - 1; ++i) {
				line_segment2<fp>::rasterize_aa(left[i], left[i + 1], SET_PIXEL_AA);
			}

			auto set_pixel = [&](int x, int y) {SET_PIXEL_AA(x, y, 1.f);};
			rasterize_scanline(left, set_pixel);
		}

		template <typename fp>
		inline std::optional<point2<fp>> curve2<fp>::find_path_intersection(const typename curve2<fp>::path_point& path1, const typename curve2<fp>::path_point& path2)
		{
			const auto& p1 = path1.first.vec();
			const auto& p2 = path2.first.vec();
			const auto& v1 = path1.second.vec();
			const auto& v2 = path2.second.vec();

			if (path1.first.distance(path2.first) <= sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(3.f)) // too close
				return {};

			auto determinant = v1.x * v2.y - v1.y * v2.x;

			auto dot = low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim>::dot(v1, v2);
			if (low_level::mathf::abs(dot) >= sixit::units::create_dimensionless_scalar<fp>(0.86f)) // angle < 30
				return point2(p1);	// build line directly from p1 to p2

			auto t1 = (v2.x * (p1.y - p2.y) - v2.y * (p1.x - p2.x)) / determinant;
			if (sixit::geometry::low_level::mathf::isfinite(t1))
				return point2(p1.x + t1 * v1.x, p1.y + t1 * v1.y);
			else
				return {};
		}

		// curv_builder implementation
		template <typename fp>
		std::vector<typename curve2_primitive<fp>::pcurve_segment> curv_builder<fp>::build(const shape_composite_cbezier2<fp>& shape, const typename curve2<fp>::path_point& pp0, 
																							const typename curve2<fp>::path_point& pp1)
		{
			std::vector<typename curve2_primitive<fp>::pcurve_segment> curve;

			hermit2<fp> hermit = build_optimized_hermit(pp0, pp1, shape.min_radius);
			auto r = hermit.min_curvature_r();
			if (r > shape.min_radius)
			{
				// one-segment cubic bezier
				cbezier2 cb = hermit.to_cubic_bezier();
				curve.push_back(std::make_unique<cbezier2<fp>>(std::move(cb)));
			}
			else
			{
				// build multi segment cubic bezier
				auto optimum_arc_line_arc = build(shape_arc_line_arc2<fp>(shape.min_radius), pp0, pp1);

				// one of arcs can be very small(less then arc angle threshold) but not both
				const arc2<fp>& arc0 = dynamic_cast<const arc2<fp>&>(*optimum_arc_line_arc[0]);
				// const line_segment2& ls = dynamic_cast<const line_segment2&>(*optimum_arc_line_arc[1]);
				const arc2<fp>& arc1 = dynamic_cast<const arc2<fp>&>(*optimum_arc_line_arc[2]);

				auto fp_05 = sixit::units::create_dimensionless_scalar<fp>(0.5f);

				typename curve2<fp>::path_point arc_point_0(arc0.track_point(fp_05), arc0.track_tangent(fp_05));
				typename curve2<fp>::path_point arc_point_1(arc1.track_point(fp_05), arc1.track_tangent(fp_05));
				if (arc0.length() < sixit::units::create_dimensionless_scalar<fp>(arc_threshold) * arc0.radius)
				{
					// make only 2 segments  - skip arc0
					curve.push_back(std::make_unique<cbezier2<fp>>(build_optimized_hermit(pp0, arc_point_1, shape.min_radius).to_cubic_bezier()));
					curve.push_back(std::make_unique<cbezier2<fp>>(build_optimized_hermit(arc_point_1, pp1, shape.min_radius).to_cubic_bezier()));
				}
				else
				if (arc1.length() < sixit::units::create_dimensionless_scalar<fp>(arc_threshold) * arc1.radius)
				{
					// make only 2 segments  - skip arc1
					curve.push_back(std::make_unique<cbezier2<fp>>(build_optimized_hermit(pp0, arc_point_0, shape.min_radius).to_cubic_bezier()));
					curve.push_back(std::make_unique<cbezier2<fp>>(build_optimized_hermit(arc_point_0, pp1, shape.min_radius).to_cubic_bezier()));
				}
				else
				{
					// common case
					curve.push_back(std::make_unique<cbezier2<fp>>(build_optimized_hermit(pp0, arc_point_0, shape.min_radius).to_cubic_bezier()));
					curve.push_back(std::make_unique<cbezier2<fp>>(build_optimized_hermit(arc_point_0, arc_point_1, shape.min_radius).to_cubic_bezier()));
					curve.push_back(std::make_unique<cbezier2<fp>>(build_optimized_hermit(arc_point_1, pp1, shape.min_radius).to_cubic_bezier()));
				}
			}

			return curve;
		}
		
		template <typename fp>
		std::vector<typename curve2_primitive<fp>::pcurve_segment> curv_builder<fp>::build(const shape_line_arc_line2<fp>& shape, const typename curve2<fp>::path_point& pp0, const typename curve2<fp>::path_point& pp1)
		{
			auto fp_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f);
			std::vector<typename curve2_primitive<fp>::pcurve_segment> curve;

			// prepare source data: points & tangents, and arc directions
			low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> p0 = pp0.first.vec();
			low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> p1 = pp1.first.vec();
			low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> t0 = pp0.second.vec();
			low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> t1 = pp1.second.vec();

			// chord
			low_level::dimensional_vector2 chord = p1 - p0;
			bool cw = chord.x * t0.y - chord.y * t0.x >= fp_0;

			// radius vectors and arc centers
			low_level::dimensional_vector2 r0 = cw ? low_level::dimensional_vector2(-t0.y, t0.x) : low_level::dimensional_vector2(t0.y,-t0.x);
			low_level::dimensional_vector2 r1 = cw ? low_level::dimensional_vector2(-t1.y, t1.x) : low_level::dimensional_vector2(t1.y,-t1.x);
			low_level::dimensional_vector2 c0 = p0 - (r0 * shape.radius);
			low_level::dimensional_vector2 c1 = p1 - (r1 * shape.radius);

			// find cross point
			low_level::dimensional_vector2 cc = c1 - c0;
			auto det = t0.x * t1.y - t0.y * t1.x;

			auto tt0 = (cc.x * t1.y - cc.y * t1.x) / det;
			auto tt1 = (t0.x * cc.y - t0.y * cc.x) / det;

			if (!sixit::geometry::low_level::mathf::isfinite(tt0) || !sixit::geometry::low_level::mathf::isfinite(tt1))
				return curve;

			if (tt0 <= fp_0 || tt1 <= fp_0)
			{
				if (cw == true)
				{
					return curve;
				}

				cw = true;

				// radius vectors and arc centers
				r0 = low_level::dimensional_vector2(-t0.y, t0.x);
				r1 = low_level::dimensional_vector2(-t1.y, t1.x);
				c0 = p0 - (r0 * shape.radius);
				c1 = p1 - (r1 * shape.radius);

				// find cross point
				cc = c1 - c0;
				tt0 = (cc.x * t1.y - cc.y * t1.x) / det;
				tt1 = (t0.x * cc.y - t0.y * cc.x) / det;
				if (tt0 <= fp_0 || tt1 <= fp_0)
				{
					return curve;
				}
			}

			low_level::dimensional_vector2 c = c0 + t0 * tt0;
			low_level::dimensional_vector2 ccp0 = p0 + t0 * tt0;
			low_level::dimensional_vector2 ccp1 = p1 - t1 * tt1;

			line_segment2<fp> ls_0(p0, ccp0);
			line_segment2<fp> ls_1(ccp1, p1);
			auto a0 = arc2<fp>::angle_from_radius_vector(r0);
			auto a1 = arc2<fp>::norm_angle(a0, arc2<fp>::angle_from_radius_vector(r1), cw);
			arc2<fp> arc(c, shape.radius, a0, a1);

			curve.push_back(std::make_unique<line_segment2<fp>>(std::move(ls_0)));
			curve.push_back(std::make_unique<arc2<fp>>(std::move(arc)));
			curve.push_back(std::make_unique<line_segment2<fp>>(std::move(ls_1)));
			return curve;
		}
		
		template <typename fp>
		std::vector<typename curve2_primitive<fp>::pcurve_segment> curv_builder<fp>::build(const shape_arc_line_arc2<fp>& shape, const typename curve2<fp>::path_point& pp0, const typename curve2<fp>::path_point& pp1)
		{
			std::vector<typename curve2_primitive<fp>::pcurve_segment> curve;

			// choose from get two guaranteed curves
			auto cw_curve = build_arc_line_arc_same_dir(pp0, pp1, true, shape.radius);
			auto cw_length = arc_line_arc_length(cw_curve);
			auto ccw_curve = build_arc_line_arc_same_dir(pp0, pp1, false, shape.radius);
			auto ccw_length = arc_line_arc_length(ccw_curve);

			std::tuple<arc2<fp>, line_segment2<fp>, arc2<fp>> optimum_ala = cw_length < ccw_length ? cw_curve : ccw_curve;
			auto optimum_length = cw_length < ccw_length ? cw_length : ccw_length;

			// build and check two optional solutions
			auto ala = build_arc_line_arc_opp_dir(pp0, pp1, true, shape.radius);
			if (ala.has_value())
			{
				auto ala_length = arc_line_arc_length(ala.value());
				if (ala_length < optimum_length)
				{
					optimum_length = ala_length;
					optimum_ala = ala.value();
				}
			}

			ala = build_arc_line_arc_opp_dir(pp0, pp1, false, shape.radius);
			if (ala.has_value())
			{
				auto ala_length = arc_line_arc_length(ala.value());
				if (ala_length < optimum_length)
				{
					optimum_length = ala_length;
					optimum_ala = ala.value();
				}
			}

			curve.push_back(std::make_unique<arc2<fp>>(std::move(std::get<0>(optimum_ala))));
			curve.push_back(std::make_unique<line_segment2<fp>>(std::move(std::get<1>(optimum_ala))));
			curve.push_back(std::make_unique<arc2<fp>>(std::move(std::get<2>(optimum_ala))));
			return curve;
		}

		template <typename fp>
		sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> curv_builder<fp>::arc_line_arc_length(const std::tuple<arc2<fp>, line_segment2<fp>, arc2<fp>> ala)
		{
			return std::get<0>(ala).length() + std::get<1>(ala).length() + std::get<2>(ala).length();
		}

		// build arc_line_arc composite with arcs in same direction
		template <typename fp>
		std::tuple<arc2<fp>, line_segment2<fp>, arc2<fp>> curv_builder<fp>::build_arc_line_arc_same_dir(const typename curve2<fp>::path_point& pp0, 
							const typename curve2<fp>::path_point& pp1, bool cw, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> r)
		{
			fp fp_0(0.f);
			fp fp_1(1.f);

			low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> p0 = pp0.first.vec();
			low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> p1 = pp1.first.vec();
			low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> t0 = pp0.second.vec();
			low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> t1 = pp1.second.vec();

			// radius vectors
			low_level::dimensional_vector2 r0 = arc2<fp>::radius_vector_from_tangent(t0, cw);
			low_level::dimensional_vector2 r1 = arc2<fp>::radius_vector_from_tangent(t1, cw);

			// arc centers
			low_level::dimensional_vector2 c0 = p0 - r0 * r;
			low_level::dimensional_vector2 c1 = p1 - r1 * r;

			// center connection line == commont tangent and common radius vectors
			low_level::dimensional_vector2 tc = (c1 - c0).normalized();
			low_level::dimensional_vector2 rc_0 = arc2<fp>::radius_vector_from_tangent(tc, cw);
			low_level::dimensional_vector2 rc_1 = arc2<fp>::radius_vector_from_tangent(tc, cw);

			// construct arcs
			auto a0_0 = arc2<fp>::angle_from_radius_vector(r0);
			auto a0_1 = arc2<fp>::norm_angle(a0_0, arc2<fp>::angle_from_radius_vector(rc_0), cw);
			arc2<fp> arc_0(c0, r, a0_0, a0_1);

			auto a1_0 = arc2<fp>::angle_from_radius_vector(rc_1);
			auto a1_1 = arc2<fp>::norm_angle(a1_0, arc2<fp>::angle_from_radius_vector(r1), cw);
			arc2<fp> arc_1(c1, r, a1_0, a1_1);

			line_segment2 ls(arc_0.track_point(sixit::units::create_dimensionless_scalar<fp>(fp_1)), 
								arc_1.track_point(sixit::units::create_dimensionless_scalar<fp>(fp_0)));
			return { arc_0, ls, arc_1};
		}

		// build arc_line_arc composite with arcs in opposite directions, cw_0 defines tthe direction(cw/ccw) for arc_0
		template <typename fp>
		std::optional<std::tuple<arc2<fp>, line_segment2<fp>, arc2<fp>>> curv_builder<fp>::build_arc_line_arc_opp_dir(const typename curve2<fp>::path_point& pp0, 
										const typename curve2<fp>::path_point& pp1, bool cw_0, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> r)
		{
			fp fp_0(0.f);
			fp fp_1(1.f);
			fp fp_2(2.f);

			low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> p0 = pp0.first.vec();
			low_level::dimensional_vector2<fp, sixit::units::length_unit::dim> p1 = pp1.first.vec();
			low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> t0 = pp0.second.vec();
			low_level::dimensional_vector2<fp, sixit::units::simple_scalar::dim> t1 = pp1.second.vec();

			// radius vectors
			low_level::dimensional_vector2 r0 = arc2<fp>::radius_vector_from_tangent(t0, cw_0);
			low_level::dimensional_vector2 r1 = arc2<fp>::radius_vector_from_tangent(t1, !cw_0);

			// arc centers
			low_level::dimensional_vector2 c0 = p0 - r0 * r;
			low_level::dimensional_vector2 c1 = p1 - r1 * r;

			// center connection line 
			low_level::dimensional_vector2 tc = c1 - c0;
			auto d = tc.magnitude();
			auto hd = d / sixit::units::create_dimensionless_scalar<fp>(fp_2);
			if (hd <= r)
				return {};

			//fp angle = arc2::angle_from_radius_vector(tc);
			auto l = low_level::mathf::sqrt(hd * hd - r * r);
			auto rot_x = l / hd;
			auto rot_y = cw_0 ?  sixit::units::create_dimensionless_scalar<fp>(fp_0) - (
				r / hd) : r / hd;
			low_level::dimensional_vector2 tangent = low_level::dimensional_vector2(tc.x * rot_x - tc.y * rot_y, tc.x * rot_y + tc.y * rot_x).normalized();
			low_level::dimensional_vector2 rt0 = arc2<fp>::radius_vector_from_tangent(tangent, cw_0);
			low_level::dimensional_vector2 rt1 = -rt0;

			auto a0_0 = arc2<fp>::angle_from_radius_vector(r0);
			auto a0_1 = arc2<fp>::norm_angle(a0_0, arc2<fp>::angle_from_radius_vector(rt0), cw_0);
			auto a1_0 = arc2<fp>::angle_from_radius_vector(rt1);
			auto a1_1 = arc2<fp>::norm_angle(a1_0, arc2<fp>::angle_from_radius_vector(r1), !cw_0);

			arc2<fp> arc_0(c0, r, a0_0, a0_1);
			arc2<fp> arc_1(c1, r, a1_0, a1_1);
			line_segment2 ls(arc_0.track_point(sixit::units::create_dimensionless_scalar<fp>(fp_1)), 
								arc_1.track_point(sixit::units::create_dimensionless_scalar<fp>(fp_0)));
			return { { arc_0, ls, arc_1 } };
		}

		template <typename fp>
		hermit2<fp> curv_builder<fp>::build_optimized_hermit(const typename curve2<fp>::path_point& p0, 
									const typename curve2<fp>::path_point& p1, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_radius)
		{
			fp fp_05(0.5f);
			low_level::dimensional_vector2 chord = p1.first.vec() - p0.first.vec();
			auto initial_tan_length = sixit::units::create_dimensionless_scalar<fp>(fp_05) * chord.magnitude();
			hermit2<fp> h(p0.first, p1.first, point2<fp>(p0.second.vec() * initial_tan_length), point2<fp>(p1.second.vec() * initial_tan_length));
			hermit2<fp> opt_h = optimize_hermit(h, min_radius);
			return opt_h;
		}

		template <typename fp>
		hermit2<fp> curv_builder<fp>::optimize_hermit(const hermit2<fp>& h, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_r)
		{
			fp fp_1(1.f);
			fp fp_100(100.f);

			fp scale_step(1.05f);
			fp sf_0(fp_1);
			fp sf_1(fp_1);

			hermit2 opt_h(h);
			auto curr_r = opt_h.min_curvature_r();

			bool app_0 = false;
			bool app_1 = false;
			while (curr_r < min_r)
			{
				// apply t0
				auto tmp = sf_0;
				sf_0 = sf_0 * scale_step;
				opt_h.v0 = point2(h.v0.vec() * sixit::units::create_dimensionless_scalar<fp>(sf_0));
				auto r = opt_h.min_curvature_r();
				if (r < curr_r)
				{
					// roll back
					sf_0 = tmp;
					opt_h.v0 = point2(h.v0.vec() * sixit::units::create_dimensionless_scalar<fp>(sf_0));
					app_0 = false;
				}
				else
				{
					curr_r = r;
					app_0 = true;
				}

				// apply t1
				tmp = sf_1;
				sf_1 = sf_1 * scale_step;
				opt_h.v1 = point2(h.v1.vec() * sixit::units::create_dimensionless_scalar<fp>(sf_1));
				r = opt_h.min_curvature_r();
				if (r < curr_r)
				{
					//roll back
					sf_1 = tmp;
					opt_h.v1 = point2(h.v1.vec() * sixit::units::create_dimensionless_scalar<fp>(sf_1));
					app_1 = false;
				}
				else
				{
					curr_r = r;
					app_1 = true;
				}

				if (!app_0 && !app_1)
					return opt_h;
				if (sf_0 > fp_100 || sf_1 > fp_100)
					return opt_h;
			}

			return opt_h;
		}

		template <typename fp>
		hermit2<fp> curv_builder<fp>::optimize_hermit2(const hermit2<fp>& h, sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim> min_r)
		{
			fp fp_1(1.f);
			fp fp_100(100.f);

			fp scale_step(1.05f);
			fp sf_0 = fp_1;
			fp sf_1 = fp_1;

			hermit2 opt_h(h);
			fp curr_r = opt_h.min_curvature_r();

			bool app_0 = false;
			bool app_1 = false;
			while (curr_r < min_r)
			{
				// apply t0
				auto tmp = sf_0;
				sf_0 = sf_0 * scale_step;
				opt_h.v0 = point2(h.v0.vec() * sf_0);
				tmp = sf_1;
				sf_1 = sf_1 * scale_step;
				opt_h.v1 = point2(h.v1.vec() * sf_1);


				auto r = opt_h.min_curvature_r();
				if (r < curr_r)
				{
					// roll back
					sf_0 = tmp;
					opt_h.v0 = point2(h.v0.vec() * sf_0);
					app_0 = false;
					//roll back
					sf_1 = tmp;
					opt_h.v1 = point2(h.v1.vec() * sf_1);
					app_1 = false;
				}
				else
				{
					curr_r = r;
					app_0 = true;
					app_1 = true;
				}

				if (!app_0 && !app_1)
					return opt_h;
				if (sf_0 > fp_100 || sf_1 > fp_100)
					return opt_h;
			}

			return opt_h;
		}
	}
}

#endif //sixit_geometry_curve_primitive_impl_h_included

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
