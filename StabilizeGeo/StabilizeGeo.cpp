// ============================================================================
//  StabilizeGeo.cpp   -   Classic 3D (DD::Image) implementation   [v003]
//
//  A single 3D node that STABILIZES geometry against a connected Axis/Camera
//  (cancels its motion on the chosen axes) OR MATCH-MOVES it (re-applies the
//  motion to revert the geo to normal). One tool, an axis input, and an
//  internal Stabilize/Match-move switch with per-axis controls.
//
//  v003 changes (over v002):
//    + Reference frame option (GeoProjectUV-style).
//      When 'use reference frame' is on, the motion is taken RELATIVE to the
//      axis pose at the chosen frame:  R(t) = A(t) * inverse(A(ref)).
//        - stabilize: at the reference frame the geo is unchanged, and every
//          other frame is locked to that exact pose -> ideal for painting /
//          projecting at one frame and keeping it consistent across the shot.
//        - matchmove : re-applies the motion relative to the reference, so a
//          stabilize->matchmove round-trip returns the geo to normal.
//
//  Inputs:  0 = geometry (required)
//           1 = axis  (Axis / Camera / Light - optional; identity if empty)
//
//  Builds against Nuke 17 commercial. The classic 3D system is deprecated in
//  Nuke 17 (deprecation warnings only) but fully functional.
// ============================================================================
#include "DDImage/GeoOp.h"
#include "DDImage/Scene.h"
#include "DDImage/Knobs.h"
#include "DDImage/Knob.h"
#include "DDImage/AxisOp.h"
#include "DDImage/GeometryList.h"
#include "DDImage/Matrix4.h"
#include "DDImage/OutputContext.h"

#include "StabilizeMath.h"

using namespace DD::Image;

static const char* const kModeNames[]  = { "stabilize", "matchmove", 0 };
static const char* const kOrderNames[] = { "ZXY", "XYZ", 0 };

class StabilizeGeo : public GeoOp
{
    int  k_mode;        // 0 = stabilize, 1 = matchmove
    int  k_order;       // rotation order (kOrderNames)

    bool k_tx, k_ty, k_tz;   // translate axes to act on
    bool k_rx, k_ry, k_rz;   // rotate axes to act on
    bool k_sx, k_sy, k_sz;   // scale axes to act on

    bool   k_use_ref;   // use a reference frame as the locked pose
    double k_ref_frame; // the reference frame number

public:
    static const Description description;
    const char* Class() const override { return description.name; }
    const char* node_help() const override {
        return "StabilizeGeo (v003)\n\n"
               "Stabilizes (locks) incoming geometry against the connected Axis/Camera on "
               "selected axes, or match-moves it (re-applies the motion to revert to normal). "
               "Connect geometry to input 0 and an Axis/Camera to the 'axis' input.\n\n"
               "mode   : stabilize = cancel the axis motion; matchmove = re-apply it.\n"
               "axes   : which translate/rotate/scale components to act on.\n\n"
               "Reference frame: when on, the chosen frame becomes the locked pose. At that "
               "frame the geo is unchanged (paint/project there); all other frames lock to it. "
               "stabilize then matchmove with the same settings returns the geo to normal.";
    }

    StabilizeGeo(Node* node) : GeoOp(node)
    {
        k_mode = 0; k_order = stab::ROT_ZXY;
        k_tx = k_ty = k_tz = true;
        k_rx = k_ry = k_rz = true;
        k_sx = k_sy = k_sz = false;
        k_use_ref = false; k_ref_frame = 1.0;
    }

    // ---- inputs -----------------------------------------------------------
    int minimum_inputs() const override { return 2; }
    int maximum_inputs() const override { return 2; }

    Op* default_input(int input) const override {
        if (input == 1) return nullptr;          // axis input is optional
        return GeoOp::default_input(input);
    }
    bool test_input(int input, Op* op) const override {
        if (input == 1) return dynamic_cast<AxisOp*>(op) != nullptr;
        return GeoOp::test_input(input, op);
    }
    const char* input_label(int input, char*) const override {
        return (input == 1) ? "axis" : "";
    }

    // ---- knobs ------------------------------------------------------------
    void knobs(Knob_Callback f) override
    {
        Enumeration_knob(f, &k_mode,  kModeNames,  "mode", "mode");
        Tooltip(f, "stabilize: cancel the axis motion so the geo locks in place.\n"
                   "matchmove: re-apply the axis motion to put the geo back to normal.");
        Enumeration_knob(f, &k_order, kOrderNames, "rot_order", "rotation order");
        Tooltip(f, "Euler order used to decompose rotation for per-axis locking. "
                   "ZXY matches Nuke's Axis default.");

        Divider(f, "Axes to act on");
        Bool_knob(f, &k_tx, "translate_x", "translate x"); SetFlags(f, Knob::STARTLINE);
        Bool_knob(f, &k_ty, "translate_y", "y");
        Bool_knob(f, &k_tz, "translate_z", "z");
        Bool_knob(f, &k_rx, "rotate_x", "rotate x");       SetFlags(f, Knob::STARTLINE);
        Bool_knob(f, &k_ry, "rotate_y", "y");
        Bool_knob(f, &k_rz, "rotate_z", "z");
        Bool_knob(f, &k_sx, "scale_x", "scale x");         SetFlags(f, Knob::STARTLINE);
        Bool_knob(f, &k_sy, "scale_y", "y");
        Bool_knob(f, &k_sz, "scale_z", "z");

        Divider(f, "Reference frame");
        Bool_knob(f, &k_use_ref, "use_reference", "use reference frame");
        Tooltip(f, "Lock to the axis pose at the reference frame. At that frame the geo is "
                   "unchanged (paint/project there); all other frames lock to it.");
        Double_knob(f, &k_ref_frame, "ref_frame", "frame");
        Tooltip(f, "The frame whose pose becomes the locked reference.");
    }

    // ---- helpers ----------------------------------------------------------
    stab::AxisMask mask() const {
        stab::AxisMask m;
        m.tx = k_tx; m.ty = k_ty; m.tz = k_tz;
        m.rx = k_rx; m.ry = k_ry; m.rz = k_rz;
        m.sx = k_sx; m.sy = k_sy; m.sz = k_sz;
        return m;
    }
    stab::RotationOrder order() const { return (stab::RotationOrder)k_order; }

    // World matrix of the connected axis at the current frame (identity if none).
    Matrix4 axisMatrix() const {
        AxisOp* ax = dynamic_cast<AxisOp*>(Op::input(1));
        if (!ax) return stab::ident();
        ax->validate(true);
        return ax->matrix();   // classic Matrix4 (deprecation warning is harmless)
    }

    // World matrix of the connected axis at an arbitrary frame.
    Matrix4 axisMatrixAt(double frame) const {
        AxisOp* ax = dynamic_cast<AxisOp*>(Op::input(1));
        if (!ax) return stab::ident();
        ax->validate(true);
        OutputContext oc = outputContext();
        oc.setFrame(frame);
        Matrix4 m;
        ax->matrixAt(oc, m);   // fills a DD::Image::Matrix4 at the given context
        return m;
    }

    Matrix4 resultMatrix() const {
        const bool stabilize = (k_mode == 0);
        if (k_use_ref) {
            Matrix4 ref = axisMatrixAt(k_ref_frame);
            return stab::buildResult(axisMatrix(), stabilize, mask(), order(), true, ref);
        }
        return stab::buildResult(axisMatrix(), stabilize, mask(), order());
    }

    // ---- geometry pipeline ------------------------------------------------
    void get_geometry_hash() override
    {
        GeoOp::get_geometry_hash();

        Matrix4 X = resultMatrix();
        Hash h;
        h.append(k_mode); h.append(k_order);
        h.append(k_tx); h.append(k_ty); h.append(k_tz);
        h.append(k_rx); h.append(k_ry); h.append(k_rz);
        h.append(k_sx); h.append(k_sy); h.append(k_sz);
        h.append(k_use_ref); h.append(k_ref_frame);
        const float* e = X.array();
        for (int i = 0; i < 16; ++i) h.append(e[i]);

        geo_hash[Group_Matrix].append(h.value());
    }

    void geometry_engine(Scene& scene, GeometryList& out) override
    {
        input0()->get_geometry(scene, out);
        const Matrix4 X = resultMatrix();
        const unsigned n = out.objects();
        for (unsigned i = 0; i < n; ++i)
            out[i].matrix = X * out[i].matrix;
    }

private:
    static Op* build(Node* node) { return new StabilizeGeo(node); }
};

const Op::Description StabilizeGeo::description("StabilizeGeo", "3D/Modify/StabilizeGeo",
                                               StabilizeGeo::build);
