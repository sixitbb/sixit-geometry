/*
Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.
This file is licensed under The 3-Clause BSD License, with full text available at the end of the file.
Contributors: Sherry Ignatchenko, Serhii Iliukhin
*/

#ifndef sixit_geometry_units_h_included
#define sixit_geometry_units_h_included

#include "sixit/core/lwa.h"

#include "sixit/dmath/numbers.h"

namespace sixit
{
namespace geometry
{

template<class fp>
class centimeters;
template<class fp>
class inches;
//template<class fp>
//class displacement3;

template <typename fp> class radians;

template<class fp>
class meters
{
  friend struct sixit::lwa::fmt::formatter<meters<fp>>;
  friend class centimeters<fp>;
  friend class inches<fp>;
  //friend class displacement3<fp>;
  //friend struct direction3<fp>;
  //friend struct direction2<fp>;

  public:
    using value_t = sixit::units::dimensional_scalar<fp, sixit::units::meter::dim>;
  
  public:
    explicit meters(fp value) : value_({ value, sixit::units::internal_constructor_of_dimensional_scalar_from_fp() }) { }
    explicit meters(centimeters<fp> value) : value_(value.value_) {}
    explicit meters(inches<fp> value) : value_(value.value_) {}

    template <typename This, typename ComparserT>
    static void read_write(This& obj, ComparserT& comparser)
    {
        sixit::rw::begin_struct<"_meters">(comparser, obj);
        sixit::rw::read_write<&meters::value_, "value_", sixit::rw::F32>(comparser);
        sixit::rw::end_struct(comparser);
    }

    value_t value_ = value_t::zero();
};

template<class fp>
class centimeters
{
  friend struct sixit::lwa::fmt::formatter<centimeters<fp>>;
  friend class meters<fp>;
  friend class inches<fp>;
  //friend struct direction3<fp>;
  //friend struct direction2<fp>;

  public:
    using value_t = sixit::units::dimensional_scalar<fp, sixit::units::meter::dim>;

  public:
    explicit centimeters(fp value) : value_({value / fp(100.f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() }) { }
    explicit centimeters(meters<fp> value) : value_(value.value_) {}
    explicit centimeters(inches<fp> value) : value_(value.value_) {}

    template <typename This, typename ComparserT>
    static void read_write(This& obj, ComparserT& comparser)
    {
        sixit::rw::begin_struct<"_centimeters">(comparser, obj);
        sixit::rw::read_write<&centimeters::value_, "value_", sixit::rw::F32>(comparser);
        sixit::rw::end_struct(comparser);
    }

  private:
    value_t value_ = value_t::zero();
};

template<class fp>
class inches
{
  friend struct sixit::lwa::fmt::formatter<inches<fp>>;
  friend class meters<fp>;
  friend class centimeters<fp>;
  //friend struct direction3<fp>;

  public:
    using value_t = sixit::units::dimensional_scalar<fp, sixit::units::meter::dim>;

  public:
    explicit inches(fp value) : value_({value / fp(0.0254f), sixit::units::internal_constructor_of_dimensional_scalar_from_fp() }) { }
    explicit inches(meters<fp> value) : value_(value.value_) {}
    explicit inches(centimeters<fp> value) : value_(value.value_) {}

    template <typename This, typename ComparserT>
    static void read_write(This& obj, ComparserT& comparser)
    {
        sixit::rw::begin_struct<"_inches">(comparser, obj);
        sixit::rw::read_write<&inches::value_, "value_", sixit::rw::F32>(comparser);
        sixit::rw::end_struct(comparser);
    }

  private:
    value_t value_ = value_t::zero();
};

struct axis_x_tag {};
struct axis_y_tag {};
struct axis_z_tag {};

template <typename fp>
class degrees
{
  friend struct sixit::lwa::fmt::formatter<degrees<fp>>;
  friend class radians<fp>;
  friend struct direction2<fp>;
  friend struct rotation3<fp>;

  public:
    explicit degrees(fp angle) : value_(angle) { }
    explicit degrees(radians<fp> angleInRadians);

    template <typename This, typename ComparserT>
    static void read_write(This& obj, ComparserT& comparser)
    {
        sixit::rw::begin_struct<"degrees">(comparser, obj);
        sixit::rw::read_write<&degrees::value_, "value_", sixit::rw::F32>(comparser);
        sixit::rw::end_struct(comparser);
    }

  private:
    fp value_ = 0.0f;
};

template <typename fp>
class radians
{
  friend struct sixit::lwa::fmt::formatter<radians<fp>>;
  friend class degrees<fp>;
  friend struct direction2<fp>;
  friend struct rotation3<fp>;
  friend struct direction3<fp>;

  public:
    explicit radians(fp angle) : value_(angle) { }
    explicit radians(degrees<fp> angleInDegrees);

    template <typename This, typename ComparserT>
    static void read_write(This& obj, ComparserT& comparser)
    {
        sixit::rw::begin_struct<"radians">(comparser, obj);
        sixit::rw::read_write<&radians::value_, "value_", sixit::rw::F32>(comparser);
        sixit::rw::end_struct(comparser);
    }

  private:
    fp value_ = 0.0f;
};

// Conversion to _meters
/*template <typename fp>
inline meters<fp>::meters(centimeters<fp> value) : value_(value.value_ * fp(0.01f)) { }
template <typename fp>
inline meters<fp>::meters(inches<fp> value) : value_(value.value_ * fp(0.0254f)) { }

// Conversion to _centimeters
template <typename fp>
inline centimeters<fp>::centimeters(meters<fp> value) : value_(value.value_ / fp(0.01f)) { }
template <typename fp>
inline centimeters<fp>::centimeters(inches<fp> value) : value_(value.value_ * fp(2.54f)) { }

// Conversion to _inches
template <typename fp>
inline inches<fp>::inches(meters<fp> value) : value_(value.value_ / fp(0.0254f)) { }
template <typename fp>
inline inches<fp>::inches(centimeters<fp> value) : value_(value.value_ / fp(2.54f)) { }*/

// Conversion to degrees
template <typename fp>
inline degrees<fp>::degrees(radians<fp> angleInRadians) : value_(angleInRadians.value_ * fp(180.0f) / fp(sixit::dmath::numbers::pi_v<float>)) { }

// Conversion to radians
template <typename fp>
inline radians<fp>::radians(degrees<fp> angleInDegrees) : value_(angleInDegrees.value_ * fp(sixit::dmath::numbers::pi_v<float>) / fp(180.0f)) { }

}; // namespace geometry


namespace geometry 
{
  using metersf = meters<float>;
  using centimetersf = centimeters<float>;
  using inchesf = inches<float>;
  using degreesf = degrees<float>;
  using radiansf = radians<float>;
}

}; // namespace sixit

#endif //sixit_geometry_units_h_included

/*
The 3-Clause BSD License

Copyright (C) 2023-2024 Six Impossible Things Before Breakfast Limited.

Contributors: Sherry Ignatchenko, Serhii Iliukhin

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
