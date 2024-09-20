/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Mykhailo Borovyk
*/

#ifndef sixit_geometry_curve_primitive_h_included
#define sixit_geometry_curve_primitive_h_included

#include "sixit/geometry/point.h"
#include "sixit/geometry/rotation.h"

namespace sixit
{
	namespace geometry
	{
		// forward declaration
		template <typename fp>
		struct curve2_primitive;

		template <typename fp>
		struct line_segment2;
		template <typename fp>
		struct arc2;

		template <typename fp>
		struct qbezier2;

		template <typename fp>
		struct cbezier2;
		
		template <typename fp>
		struct curve2;
		
		template <typename fp>
		struct curve2_primitive
		{
			typedef std::unique_ptr<curve2_primitive<fp>> pcurve_segment;
			enum class primitive_type :int32_t 
			{  
				base = 0,
				line_segment = 1,
				arc = 2,
				qbezier = 3,
				cbezier = 4,
				curve = 5
			};

			using class_value = sixit::units::dimensional_scalar<fp, sixit::units::length_unit::dim>;

			virtual inline primitive_type get_primitive_type() const { return primitive_type::base; } 
			virtual inline class_value length() const 
			{ 
				auto ds_0 = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(0.f);
				return ds_0; 
			}
			virtual inline point2<fp> track_point(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const = 0;
			virtual inline direction2<fp> track_tangent(sixit::units::dimensional_scalar<fp, sixit::units::simple_scalar::dim> t) const = 0;
			virtual inline bounds2<fp> bounds() const = 0;

			virtual inline class_value min_curvature_radius() const
			{
				auto ds_inf = sixit::units::for_import_only_make_dimensional_scalar<fp, sixit::units::meter>(low_level::mathf::infinity);
				return ds_inf;
			}
			virtual inline std::pair<point2<fp>, rotation2<fp>> track(class_value x) const 
			{
				auto fp_0 = sixit::units::create_dimensionless_scalar<fp>(0.f);
				auto fp_1 = sixit::units::create_dimensionless_scalar<fp>(1.f);
				auto t = x / length();

				direction2<fp> dir = direction2<fp>(fp_0, fp_1);
				return std::pair<point2<fp>, rotation2<fp>>({ track_point(t), rotation2<fp>(dir, track_tangent(t))});
			}
			
			// virtual intersection methods
			virtual inline void intersection(std::vector<point2<fp>>&, const pcurve_segment&) const {}
			virtual inline void intersection(std::vector<point2<fp>>&, const line2<fp>&) const {}
			virtual inline void intersection(std::vector<point2<fp>>&, const line_segment2<fp>&) const {}
			virtual inline void intersection(std::vector<point2<fp>>&, const arc2<fp>&) const {}
			virtual inline void intersection(std::vector<point2<fp>>&, const qbezier2<fp>&) const {}
			virtual inline void intersection(std::vector<point2<fp>>&, const cbezier2<fp>&) const {}
			virtual inline void intersection(std::vector<point2<fp>>&, const curve2<fp>&) const {}

			virtual inline bool intersect(const pcurve_segment&) const { return false; }
			virtual inline bool intersect(const line2<fp>&) const { return false; }
			virtual inline bool intersect(const line_segment2<fp>&) const { return false; }
			virtual inline bool intersect(const arc2<fp>&) const { return false; }
			virtual inline bool intersect(const qbezier2<fp>&) const { return false; }
			virtual inline bool intersect(const cbezier2<fp>&) const { return false; }
			virtual inline bool intersect(const curve2<fp>&) const { return false; }

			[[nodiscard]] virtual inline bool _for_test_only_approximate_eq(const curve2_primitive&, size_t) const
			{
				return false;
			}

            virtual ~curve2_primitive() = default;

		};

		// for serialization
		//// TODO: refactor to AofSTRUCT after TR-2362
		template <typename P, size_t N>
		struct vlo_points
		{
			std::vector<P>  points;

			vlo_points() {}
			vlo_points(const std::array<P, N>& a) : points(a.begin(), a.end()) {}

			template<typename This, typename ComparserT>
			static void read_write(This& obj, ComparserT& comparser)
			{
				sixit::rw::begin_struct<"vlo_points">(comparser, obj);
				sixit::rw::read_write<&vlo_points::points, "points", sixit::rw::VofSTRUCT>(comparser);
				sixit::rw::end_struct(comparser);
			}
		};

		//// TODO: refactor to AofSTRUCT after TR-2362
		template <typename ValueType, size_t N, typename ComparserT>
		struct vlo_points_converter : public ComparserT::aliases::template is_validating_converter<void>
		{
			using ConvertedType = vlo_points<ValueType, N>;
			auto create() { return ConvertedType(); }

			auto member2value(const auto& val)
			{
				ConvertedType converted = ConvertedType(val);
				return typename ComparserT::aliases::template conversion_result<ConvertedType>(ComparserT::aliases::qualifier_outcomes::converted, converted);
			}

			auto value2member(auto& val)
			{
				std::array<ValueType, N> m;
				for (size_t i = 0; i < val.points.size(); i++)
					m[i] = val.points[i];

				return std::make_tuple(true, std::move(m));
			}
		};
	}

namespace geometry 
{
  using curve2_primitivef = curve2_primitive<float>;
}

}
#endif //sixit_geometry_curve_primitive_h_included

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
