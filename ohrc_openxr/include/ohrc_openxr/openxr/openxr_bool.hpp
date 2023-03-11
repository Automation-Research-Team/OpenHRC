// *********** THIS FILE IS GENERATED - DO NOT EDIT ***********
//     See cpp_generator.py for modifications
// ************************************************************

/*
** Copyright (c) 2017-2021 The Khronos Group Inc.
** Copyright (c) 2019-2021 Collabora, Ltd.
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
**
** ---- Exceptions to the Apache 2.0 License: ----
**
** As an exception, if you use this Software to generate code and portions of
** this Software are embedded into the generated code as a result, you may
** redistribute such product without providing attribution as would otherwise
** be required by Sections 4(a), 4(b) and 4(d) of the License.
**
** In addition, if you combine or link code generated by this Software with
** software that is licensed under the GPLv2 or the LGPL v2.0 or 2.1
** ("`Combined Software`") and if a court of competent jurisdiction determines
** that the patent provision (Section 3), the indemnity provision (Section 9)
** or other Section of the License conflicts with the conditions of the
** applicable GPL or LGPL license, you may retroactively and prospectively
** choose to deem waived or otherwise exclude such Section(s) of the License,
** but only in their entirety and only with respect to the Combined Software.
**
*/

/*
** This header is generated from the Khronos OpenXR XML API Registry.
**
*/
#ifndef OPENXR_BOOL_HPP_
#define OPENXR_BOOL_HPP_
/**
 * @file
 * @brief Contains a type-safe C++ projection of XrBool32
 *
 * @see xr::Bool32
 * @ingroup wrappers
 */

#include <openxr/openxr.h>

#if !defined(OPENXR_HPP_INLINE)
#if defined(__clang___)
#if __has_attribute(always_inline)
#define OPENXR_HPP_INLINE __attribute__((always_inline)) __inline__
#else
#define OPENXR_HPP_INLINE inline
#endif
#elif defined(__GNUC__)
#define OPENXR_HPP_INLINE __attribute__((always_inline)) __inline__
#elif defined(_MSC_VER)
#define OPENXR_HPP_INLINE inline
#else
#define OPENXR_HPP_INLINE inline
#endif
#endif  // !OPENXR_HPP_INLINE

#if !defined(OPENXR_HPP_CONSTEXPR)
#if defined(_MSC_VER) && (_MSC_VER <= 1800)
#define OPENXR_HPP_CONSTEXPR
#else
#define OPENXR_HPP_CONSTEXPR constexpr
#endif
#endif  // !OPENXR_HPP_CONSTEXPR

#if !defined(OPENXR_HPP_SWITCH_CONSTEXPR)
//! @todo set this to constexpr in c++14
#define OPENXR_HPP_SWITCH_CONSTEXPR
#endif  // !OPENXR_HPP_SWITCH_CONSTEXPR

#if !defined(OPENXR_HPP_NAMESPACE)
#define OPENXR_HPP_NAMESPACE xr
#endif  // !OPENXR_HPP_NAMESPACE
namespace OPENXR_HPP_NAMESPACE {
/*!
 * @brief XrBool32 wrapper class
 *
 * @see <https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#XrBool32>
 *
 * @xrentity{XrBool32}
 * @ingroup wrappers
 */
class Bool32 {
public:
  /*!
   * @name Constructors, assignment, and conversions
   * @{
   */
  //! Default constructor.
  OPENXR_HPP_CONSTEXPR Bool32() noexcept = default;

  //! Explicit constructor from raw XrBool32 value
  OPENXR_HPP_CONSTEXPR explicit Bool32(XrBool32 v) noexcept : val_(v) {}

  //! Implicitly construct from bool
  OPENXR_HPP_CONSTEXPR Bool32(bool val) noexcept : val_(val ? XR_TRUE : XR_FALSE) {}

  //! Implicitly assign from bool
  Bool32 &operator=(bool val) noexcept {
    val_ = val ? XR_TRUE : XR_FALSE;
    return *this;
  }

  //! Create from a raw XrBool32. Exists because construction from integral types is weird.
  static OPENXR_HPP_CONSTEXPR Bool32 fromRaw(XrBool32 val) noexcept { return {val == XR_TRUE}; }
  //! @}
  //! True if this value is XR_TRUE
  explicit OPENXR_HPP_CONSTEXPR operator bool() const noexcept { return val_ == XR_TRUE; }

  //! Unary negation: True if this Bool32 is not XR_TRUE
  OPENXR_HPP_CONSTEXPR bool operator!() const noexcept { return val_ != XR_TRUE; }
  /*!
   * @name Raw XrBool32 manipulation
   * @{
   */
  //! Gets the raw XrBool32 value.
  OPENXR_HPP_CONSTEXPR XrBool32 get() const noexcept { return val_; }
  /*!
   * @brief Returns the address of the raw XrBool32 value,
   * for use in creation/assignment.
   */
  XrBool32 *put() noexcept { return &val_; }
  //! @}

private:
  XrBool32 val_{};
};

static_assert(sizeof(XrBool32) == sizeof(Bool32), "Original type and wrapper have different size!");

/*!
 * @brief Free function for getting the raw XrBool32 from an Bool32 value.
 *
 * @found_by_adl
 * @see Bool32::get()
 * @relates Bool32
 * @ingroup utility_accessors
 */
OPENXR_HPP_CONSTEXPR OPENXR_HPP_INLINE XrBool32 get(Bool32 v) noexcept { return v.get(); }
/*!
 * @brief Free function for getting the address of the raw XrBool32 from an Bool32 value.
 *
 * @found_by_adl
 * @see Bool32::put( )
 * @relates Bool32
 * @ingroup utility_accessors
 */
static OPENXR_HPP_INLINE XrBool32 *put(Bool32 &v) noexcept { return v.put(); }
//! @brief `==` comparison between Bool32 values.
//! @relates Bool32
OPENXR_HPP_CONSTEXPR OPENXR_HPP_INLINE bool operator==(Bool32 lhs, Bool32 rhs) noexcept {
  return lhs.get() == rhs.get();
}
//! @brief `!=` comparison between Bool32 values.
//! @relates Bool32
OPENXR_HPP_CONSTEXPR OPENXR_HPP_INLINE bool operator!=(Bool32 lhs, Bool32 rhs) noexcept {
  return lhs.get() != rhs.get();
}
//! @brief `==` comparison between Bool32 and raw XrBool32.
//! @relates Bool32
OPENXR_HPP_CONSTEXPR OPENXR_HPP_INLINE bool operator==(Bool32 lhs, XrBool32 rhs) noexcept {
  return lhs.get() == rhs;
}
//! @brief `==` comparison between raw XrBool32 and Bool32.
//! @relates Bool32
OPENXR_HPP_CONSTEXPR OPENXR_HPP_INLINE bool operator==(XrBool32 lhs, Bool32 rhs) noexcept {
  return lhs == rhs.get();
}
//! @brief `!=` comparison between Bool32 and raw XrBool32.
//! @relates Bool32
OPENXR_HPP_CONSTEXPR OPENXR_HPP_INLINE bool operator!=(Bool32 lhs, XrBool32 rhs) noexcept {
  return lhs.get() != rhs;
}
//! @brief `!=` comparison between raw XrBool32 and Bool32.
//! @relates Bool32
OPENXR_HPP_CONSTEXPR OPENXR_HPP_INLINE bool operator!=(XrBool32 lhs, Bool32 rhs) noexcept {
  return lhs != rhs.get();
}
}  // namespace OPENXR_HPP_NAMESPACE

#endif  // ifndef OPENXR_BOOL_HPP_