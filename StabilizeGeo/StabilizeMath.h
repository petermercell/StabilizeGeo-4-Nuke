// ============================================================================
//  StabilizeMath.h
//  Portable transform math for the StabilizeGeo / StabilizeGeomOp plugin.
//
//  This header is intentionally kept FREE of any 3D-system specifics so the
//  exact same math is reused by:
//    * StabilizeGeo.cpp      (Classic 3D, DD::Image::GeoOp)
//    * StabilizeGeomOp.cpp   (New USD 3D, ModifyGeomOp)
//
//  It depends only on DD::Image::Matrix4 / Vector3 for the public-facing
//  helpers, plus <cmath>. The decompose / recompose routines are written by
//  hand (rather than calling Matrix4::decompose) so the behaviour is explicit,
//  testable, and identical across both ports.  See StabilizeMathTest.cpp for a
//  standalone numerical verification of every formula in this file.
//
//  CONVENTION
//  ----------
//  * Column-vector convention:  v' = M * v
//  * DD::Image::Matrix4 member aRC  => row R, column C  (translation in a03/a13/a23)
//  * Rotations are intrinsic, composed in the selected RotationOrder.
//  * A "rigid + uniform/non-uniform scale" model is assumed (no shear). Tracked
//    cameras/axes satisfy this; if shear is present it is dropped.
// ============================================================================
#ifndef STABILIZE_MATH_H
#define STABILIZE_MATH_H

#include <cmath>
#include "DDImage/Matrix4.h"
#include "DDImage/Vector3.h"

namespace stab {

using DD::Image::Matrix4;
using DD::Image::Vector3;

// Rotation orders. Index order matches the Enumeration_knob list in the .cpp.
enum RotationOrder { ROT_ZXY = 0, ROT_XYZ = 1 };

// Which transform components to stabilize ("cancel"). Unchecked axes pass through.
struct AxisMask {
    bool tx = true,  ty = true,  tz = true;   // translate
    bool rx = true,  ry = true,  rz = true;   // rotate
    bool sx = false, sy = false, sz = false;  // scale (off by default)
};

// ---------------------------------------------------------------------------
//  Low-level matrix builders (manual, so we never rely on uncertain helper
//  method names in a particular NDK version).
// ---------------------------------------------------------------------------
inline Matrix4 ident() { Matrix4 m; m.makeIdentity(); return m; }

inline Matrix4 mTranslate(float x, float y, float z) {
    Matrix4 m; m.makeIdentity();
    m.a03 = x; m.a13 = y; m.a23 = z;
    return m;
}
inline Matrix4 mScale(float x, float y, float z) {
    Matrix4 m; m.makeIdentity();
    m.a00 = x; m.a11 = y; m.a22 = z;
    return m;
}
inline Matrix4 mRotX(float a) {
    Matrix4 m; m.makeIdentity();
    const float c = std::cos(a), s = std::sin(a);
    m.a11 = c; m.a12 = -s; m.a21 = s; m.a22 = c;
    return m;
}
inline Matrix4 mRotY(float a) {
    Matrix4 m; m.makeIdentity();
    const float c = std::cos(a), s = std::sin(a);
    m.a00 = c; m.a02 = s; m.a20 = -s; m.a22 = c;
    return m;
}
inline Matrix4 mRotZ(float a) {
    Matrix4 m; m.makeIdentity();
    const float c = std::cos(a), s = std::sin(a);
    m.a00 = c; m.a01 = -s; m.a10 = s; m.a11 = c;
    return m;
}

inline Matrix4 rotFromEuler(RotationOrder order, float rx, float ry, float rz) {
    // Intrinsic composition. v' = (first listed) * ... * (last listed) * v
    if (order == ROT_XYZ) return mRotX(rx) * mRotY(ry) * mRotZ(rz);
    /* ROT_ZXY (Nuke's Axis default) */ return mRotZ(rz) * mRotX(rx) * mRotY(ry);
}

// ---------------------------------------------------------------------------
//  Decomposed transform.
// ---------------------------------------------------------------------------
struct TRS {
    Vector3 t {0,0,0};   // translation
    Vector3 r {0,0,0};   // euler radians, in the given order
    Vector3 s {1,1,1};   // scale
    RotationOrder order = ROT_ZXY;
};

inline float vlen(float x, float y, float z) { return std::sqrt(x*x + y*y + z*z); }

// Decompose M into T * R * S for the requested rotation order.
inline TRS decompose(const Matrix4& M, RotationOrder order) {
    TRS o; o.order = order;

    // Translation = last column.
    o.t = Vector3(M.a03, M.a13, M.a23);

    // Basis columns (image of x/y/z axes).
    float c0[3] = { M.a00, M.a10, M.a20 };
    float c1[3] = { M.a01, M.a11, M.a21 };
    float c2[3] = { M.a02, M.a12, M.a22 };

    float sx = vlen(c0[0], c0[1], c0[2]);
    float sy = vlen(c1[0], c1[1], c1[2]);
    float sz = vlen(c2[0], c2[1], c2[2]);
    if (sx == 0) sx = 1e-8f;
    if (sy == 0) sy = 1e-8f;
    if (sz == 0) sz = 1e-8f;
    o.s = Vector3(sx, sy, sz);

    // Pure rotation matrix R = M with columns normalized. r(i,j) below.
    const float r00 = c0[0]/sx, r10 = c0[1]/sx, r20 = c0[2]/sx;
    const float r01 = c1[0]/sy, r11 = c1[1]/sy, r21 = c1[2]/sy;
    const float r02 = c2[0]/sz, r12 = c2[1]/sz, r22 = c2[2]/sz;

    auto clampf = [](float v){ return v < -1.f ? -1.f : (v > 1.f ? 1.f : v); };

    if (order == ROT_XYZ) {
        // R = Rx*Ry*Rz  =>  r02 = sin(ry)
        o.r.y = std::asin(clampf(r02));
        if (std::fabs(r02) < 0.9999995f) {
            o.r.x = std::atan2(-r12, r22);
            o.r.z = std::atan2(-r01, r00);
        } else { // gimbal lock
            o.r.x = std::atan2(r21, r11);
            o.r.z = 0.f;
        }
    } else { // ROT_ZXY  (R = Rz*Rx*Ry)  =>  r21 = sin(rx)
        o.r.x = std::asin(clampf(r21));
        if (std::fabs(r21) < 0.9999995f) {
            o.r.y = std::atan2(-r20, r22);
            o.r.z = std::atan2(-r01, r11);
        } else { // gimbal lock
            o.r.y = std::atan2(r02, r00);
            o.r.z = 0.f;
        }
    }
    return o;
}

inline Matrix4 recompose(const TRS& d) {
    return mTranslate(d.t.x, d.t.y, d.t.z)
         * rotFromEuler(d.order, d.r.x, d.r.y, d.r.z)
         * mScale(d.s.x, d.s.y, d.s.z);
}

// ---------------------------------------------------------------------------
//  Build the FILTERED motion matrix: keep only the checked (stabilized) axes,
//  reset everything else to neutral (0 translate / 0 rotate / 1 scale).
// ---------------------------------------------------------------------------
inline Matrix4 filterAxes(const Matrix4& motion, const AxisMask& m, RotationOrder order) {
    TRS d = decompose(motion, order);
    TRS f; f.order = order;
    f.t = Vector3(m.tx ? d.t.x : 0.f, m.ty ? d.t.y : 0.f, m.tz ? d.t.z : 0.f);
    f.r = Vector3(m.rx ? d.r.x : 0.f, m.ry ? d.r.y : 0.f, m.rz ? d.r.z : 0.f);
    f.s = Vector3(m.sx ? d.s.x : 1.f, m.sy ? d.s.y : 1.f, m.sz ? d.s.z : 1.f);
    return recompose(f);
}

// ---------------------------------------------------------------------------
//  The single entry point used by both ops.
//    stabilize == true   -> returns inverse(filtered)   (cancels the motion)
//    stabilize == false  -> returns filtered            (re-applies / matchmove)
//
//  If a reference matrix is supplied (use_reference), the motion is taken
//  relative to that pose: motion_rel = motion * inverse(reference).
// ---------------------------------------------------------------------------
inline Matrix4 buildResult(const Matrix4& motion,
                           bool stabilize,
                           const AxisMask& mask,
                           RotationOrder order,
                           bool use_reference = false,
                           const Matrix4& reference = Matrix4())
{
    Matrix4 m = motion;
    if (use_reference) {
        Matrix4 refInv = reference; refInv = refInv.inverse();
        m = motion * refInv;
    }
    Matrix4 filtered = filterAxes(m, mask, order);
    return stabilize ? filtered.inverse() : filtered;
}

} // namespace stab

#endif // STABILIZE_MATH_H
