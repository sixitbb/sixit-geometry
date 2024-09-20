/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Volodymyr Melnychuk
*/

#ifndef sixit_geometry_low_level_formatters_h_included
#define sixit_geometry_low_level_formatters_h_included

#include "sixit/core/lwa.h"

#include "sixit/geometry/low_level/forward_declaration.h"
#include "sixit/geometry/low_level/vector.h"
#include "sixit/geometry/low_level/quaternion.h"
#include "sixit/geometry/low_level/matrix.h"
#include "sixit/geometry/sixit_mathf.h"
#include "sixit/geometry/line.h"
#include "sixit/geometry/line_segment.h"
#include "sixit/geometry/rotation.h"
#include "sixit/geometry/units.h"

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::low_level::vector3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::low_level::vector3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "v3({:.3f}, {:.3f}, {:.3f})", q.x(), q.y(), q.z())
              : sixit::lwa::fmt::format_to(ctx.out(), "v3({:.3e}, {:.3e}, {:.3e})", q.x(), q.y(), q.z());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::low_level::vector2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::low_level::vector2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "v2({:.3f}, {:.3f})",q.x, q.y)
              : sixit::lwa::fmt::format_to(ctx.out(), "v2({:.3e}, {:.3e})",q.x, q.y);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::direction3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::direction3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "d3({:.3f}, {:.3f}, {:.3f})", q.vec().x(), q.vec().y(), q.vec().z())
              : sixit::lwa::fmt::format_to(ctx.out(), "d3({:.3e}, {:.3f}, {:.3f})", q.vec().x(), q.vec().y(), q.vec().z());
  }
};

template <>
struct sixit::lwa::fmt::formatter<sixit::geometry::tangent<float>>
{
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
  {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e'))
        presentation = *it++;
    if (it != end && *it != '}')
        throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::tangent<float>& q, FormatContext& ctx) const -> decltype(ctx.out())
  {
    return presentation == 'f' ? sixit::lwa::fmt::format_to(ctx.out(), "t({:.3f}, {:.3f}, {:.3f}, {})", q.dir3_.vec().x(),
                                                q.dir3_.vec().y(), q.dir3_.vec().z(), q.handedness_)
                               : sixit::lwa::fmt::format_to(ctx.out(), "t({:.3e}, {:.3f}, {:.3f}, {})", q.dir3_.vec().x(),
                                                q.dir3_.vec().y(), q.dir3_.vec().z(), q.handedness_);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::direction2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::direction2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "d2({:.3f}, {:.3f})",q.vec().x, q.vec().y)
              : sixit::lwa::fmt::format_to(ctx.out(), "d2({:.3e}, {:.3f})",q.vec().x, q.vec().y);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::direction1<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::direction1<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "d1({:.3f})",q.x)
              : sixit::lwa::fmt::format_to(ctx.out(), "d1({:.3e})",q.x);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::point3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::point3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "p3({:.3f}, {:.3f}, {:.3f})",q.vec().x(), q.vec().y(), q.vec().z())
              : sixit::lwa::fmt::format_to(ctx.out(), "p3({:.3e}, {:.3e}, {:.3e})",q.vec().x(), q.vec().y(), q.vec().z());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::point2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::point2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "p2({:.3f}, {:.3f})",q.vec().x, q.vec().y)
              : sixit::lwa::fmt::format_to(ctx.out(), "p2({:.3e}, {:.3e})",q.vec().x, q.vec().y);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::point1<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::point1<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "p1({:.3f})", q.x)
              : sixit::lwa::fmt::format_to(ctx.out(), "p1({:.3e})", q.x);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::line3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::line3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "l3({}, {})", q.p1(), q.p2())
              : sixit::lwa::fmt::format_to(ctx.out(), "l3({}, {})", q.p1(), q.p2());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::line2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::line2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "l2({}, {})", q.p1(), q.p2())
              : sixit::lwa::fmt::format_to(ctx.out(), "l2({}, {})", q.p1(), q.p2());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::line1<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::line1<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "l1({}, {})", q.p1(), q.p2())
              : sixit::lwa::fmt::format_to(ctx.out(), "l1({}, {})", q.p1(), q.p2());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::ray3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::ray3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ray3({}, {})", q.origin, q.direction)
              : sixit::lwa::fmt::format_to(ctx.out(), "ray3({}, {})", q.origin, q.direction);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::ray2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::ray2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ray2({}, {})", q.origin_data, q.direction_data)
              : sixit::lwa::fmt::format_to(ctx.out(), "ray2({}, {})", q.origin_data, q.direction_data);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::bounds3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::bounds3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "bb3({}, {})", q.minn, q.maxx)
              : sixit::lwa::fmt::format_to(ctx.out(), "bb3({}, {})", q.minn, q.maxx);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::bounds2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::bounds2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "bb2({}, {})", q.minn, q.maxx)
              : sixit::lwa::fmt::format_to(ctx.out(), "bb2({}, {})", q.minn, q.maxx);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::bounds1<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::bounds1<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "bb1({}, {})", q.minn, q.maxx)
              : sixit::lwa::fmt::format_to(ctx.out(), "bb1({}, {})", q.minn, q.maxx);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::rotation3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::rotation3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    auto a = q.euler_angles();
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "r3({}, {}, {})", a.x(), a.y(), a.z())
              : sixit::lwa::fmt::format_to(ctx.out(), "r3({}, {}, {})", a.x(), a.y(), a.z());
  }
};


template <> struct sixit::lwa::fmt::formatter<sixit::geometry::rotation2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::rotation2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    auto a = q.euler_angle();
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "r2({})", a)
              : sixit::lwa::fmt::format_to(ctx.out(), "r2({})", a);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::translation3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::translation3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "t3({:.3f}, {:.3f}, {:.3f})",q.vec.x(), q.vec.y(), q.vec.z())
              : sixit::lwa::fmt::format_to(ctx.out(), "t3({:.3e}, {:.3e}, {:.3e})",q.vec.x(), q.vec.y(), q.vec.z());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::translation2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::translation2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "t2({:.3f}, {:.3f})",q.vec.x, q.vec.y)
              : sixit::lwa::fmt::format_to(ctx.out(), "t2({:.3e}, {:.3e})",q.vec.x, q.vec.y);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::scale3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::scale3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "s3({:.3f}, {:.3f}, {:.3f})",q.vec.x(), q.vec.y(), q.vec.z())
              : sixit::lwa::fmt::format_to(ctx.out(), "s3({:.3e}, {:.3e}, {:.3e})",q.vec.x(), q.vec.y(), q.vec.z());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::scale2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::scale2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "s3({:.3f}, {:.3f})",q.vec.x, q.vec.y)
              : sixit::lwa::fmt::format_to(ctx.out(), "s3({:.3e}, {:.3e})",q.vec.x, q.vec.y);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::trs3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::trs3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
#ifdef TRS_AS_MATRIX4X4
    auto [translation, rotation, scale]  = q.trs();
      return presentation == 'f'
          ? sixit::lwa::fmt::format_to(ctx.out(), "trs3({}, {}, {})", translation, rotation, scale)
          : sixit::lwa::fmt::format_to(ctx.out(), "trs3({}, {}, {})", translation, rotation, scale);
#else
      return presentation == 'f'
          ? sixit::lwa::fmt::format_to(ctx.out(), "trs3({}, {}, {})", q.translation, q.rotation, q.scale)
          : sixit::lwa::fmt::format_to(ctx.out(), "trs3({}, {}, {})", q.translation, q.rotation, q.scale);
#endif
   
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::trs2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::trs2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "trs2({}, {}, {})", q.translation, q.rotation, q.scale)
              : sixit::lwa::fmt::format_to(ctx.out(), "trs2({}, {}, {})", q.translation, q.rotation, q.scale);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::plane3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::plane3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "plane3({}, {})",q.normal, q.point)
              : sixit::lwa::fmt::format_to(ctx.out(), "plane3({}, {})",q.normal, q.point);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::triangle3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::triangle3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "triangle3({}, {}, {})", q.points[0], q.points[1], q.points[2])
              : sixit::lwa::fmt::format_to(ctx.out(), "triangle3({}, {}, {})", q.points[0], q.points[1], q.points[2]);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::triangle2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::triangle2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "triangle2({}, {}, {})", q.points[0], q.points[1], q.points[2])
              : sixit::lwa::fmt::format_to(ctx.out(), "triangle2({}, {}, {})", q.points[0], q.points[1], q.points[2]);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::line_segment3<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::line_segment3<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ls3({}, {})", q.p1(), q.p2())
              : sixit::lwa::fmt::format_to(ctx.out(), "ls3({}, {})", q.p1(), q.p2());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::line_segment2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::line_segment2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ls2({}, {})", q.p1(), q.p2())
              : sixit::lwa::fmt::format_to(ctx.out(), "ls2({}, {})", q.p1(), q.p2());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::line_segment1<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::line_segment1<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ls1({}, {})", q.p1(), q.p2())
              : sixit::lwa::fmt::format_to(ctx.out(), "ls1({}, {})", q.p1(), q.p2());
  }
};

template <typename RefP3ConceptT> struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_point3<float, RefP3ConceptT>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::indexed_point3<float, RefP3ConceptT>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ip3({:.3f}, {:.3f}, {:.3f})",q.vec().x, q.vec().y, q.vec().z)
              : sixit::lwa::fmt::format_to(ctx.out(), "ip3({:.3e}, {:.3e}, {:.3e})",q.vec().x, q.vec().y, q.vec().z);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_point2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::indexed_point2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ip2({:.3f}, {:.3f})",q.vec().x, q.vec().y)
              : sixit::lwa::fmt::format_to(ctx.out(), "ip2({:.3e}, {:.3e})",q.vec().x, q.vec().y);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_point1<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::indexed_point1<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ip1({:.3f})", q.vec())
              : sixit::lwa::fmt::format_to(ctx.out(), "ip1({:.3e})", q.vec());
  }
};

template <typename RefP3ConceptT> struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_line_segment3<float, RefP3ConceptT>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::indexed_line_segment3<float, RefP3ConceptT>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ils3({}, {})", q.get_point(0), q.get_point(1))
              : sixit::lwa::fmt::format_to(ctx.out(), "ils3({}, {})", q.get_point(0), q.get_point(1));
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_line_segment2<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::indexed_line_segment2<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ils2({}, {})", q.get_point(0), q.get_point(1))
              : sixit::lwa::fmt::format_to(ctx.out(), "ils2({}, {})", q.get_point(0), q.get_point(1));
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_line_segment1<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::indexed_line_segment1<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "ils1({}, {})", q.get_point(0), q.get_point(1))
              : sixit::lwa::fmt::format_to(ctx.out(), "ils1({}, {})", q.get_point(0), q.get_point(1));
  }
};


template <typename RefP3ConceptT> struct sixit::lwa::fmt::formatter<sixit::geometry::indexed_triangle3<float, RefP3ConceptT>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }
  template <typename FormatContext>
  auto format(const sixit::geometry::indexed_triangle3<float, RefP3ConceptT>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "itriangle3({}, {}, {})", q.get_point(0), q.get_point(1), q.get_point(2))
              : sixit::lwa::fmt::format_to(ctx.out(), "itriangle3({}, {}, {})", q.get_point(0), q.get_point(1), q.get_point(2));
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::low_level::matrix4x4<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::low_level::matrix4x4<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "m4x4({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f})", q(0,0), q(0,1), q(0,2), q(0,3), q(1, 0), q(1,1), q(1, 2), q(1, 3), q(2, 0), q(2, 1), q(2, 2), q(2, 3), q(3, 0), q(3, 1), q(3, 2), q(3, 3))
              : sixit::lwa::fmt::format_to(ctx.out(), "m4x4({:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e}, {:.3e})", q(0,0), q(0,1), q(0,2), q(0,3), q(1, 0), q(1,1), q(1, 2), q(1, 3), q(2, 0), q(2, 1), q(2, 2), q(2, 3), q(3, 0), q(3, 1), q(3, 2), q(3, 3));
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::low_level::quaternion<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::low_level::quaternion<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "q({:.3f}, {:.3f}, {:.3f}, {:.3f})", q.w(), q.x(), q.y(), q.z())
              : sixit::lwa::fmt::format_to(ctx.out(), "q({:.3e}, {:.3e}, {:.3e}, {:.3e})", q.w(), q.x(), q.y(), q.z());
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::degrees<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::degrees<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "degrees({:.3f})", q.value_)
              : sixit::lwa::fmt::format_to(ctx.out(), "degrees({:.3e})", q.value_);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::radians<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::radians<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "radians({:.3f})", q.value_)
              : sixit::lwa::fmt::format_to(ctx.out(), "radians({:.3e})", q.value_);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::meters<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::meters<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "meters({:.3f})", q.value_)
              : sixit::lwa::fmt::format_to(ctx.out(), "meters({:.3e})", q.value_);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::centimeters<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::centimeters<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "centimeters({:.3f})", q.value_)
              : sixit::lwa::fmt::format_to(ctx.out(), "centimeters({:.3e})", q.value_);
  }
};

template <> struct sixit::lwa::fmt::formatter<sixit::geometry::inches<float>> {
  char presentation = 'f';
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'e')) presentation = *it++;
    if (it != end && *it != '}') throw format_error("invalid format");
    return it;
  }

  template <typename FormatContext>
  auto format(const sixit::geometry::inches<float>& q, FormatContext& ctx) const -> decltype(ctx.out()) {
    return presentation == 'f'
              ? sixit::lwa::fmt::format_to(ctx.out(), "inches({:.3f})", q.value_)
              : sixit::lwa::fmt::format_to(ctx.out(), "inches({:.3e})", q.value_);
  }
};

#endif //sixit_geometry_low_level_formatters_h_included

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