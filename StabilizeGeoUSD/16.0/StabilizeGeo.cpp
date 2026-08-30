// ============================================================================
//  StabilizeGeo.cpp  -  NEW 3D / USD system (ModifyGeomOp)   [v007]
//
//  USD-pipeline StabilizeGeo. Stabilizes / match-moves mesh prims by baking the
//  points with a transform, on per-axis components, with an optional reference
//  frame.  Reuses the verified StabilizeMath.h math (unchanged).
//
//  Motion source: an AXIS/CAMERA wired to INPUT 1, like the classic node.
//
//  v007 fix (over v006):
//    * Engine constructor no longer names GeomOpNode. That type does not exist
//      in the Nuke 16.0 NDK, where GeomOp::BuildEngine<E>() expands to
//          return [](Op* parent) { return new Engine(parent); };
//      i.e. the engine is constructed from a plain DD::Image::Op*. Nuke 17.x
//      passes a GeomOpNode*. A template constructor accepts either and lets
//      ModifyEngine's own constructor resolve the conversion, so one source
//      tree builds on both 16.0 (Linux) and 17.1 (macOS).
//    * AxisOp::matrix() kept (it returns DD::Image::Matrix4, which is what
//      StabilizeMath.h operates on) but its -Wdeprecated-declarations warning
//      is suppressed at the call site. The suggested replacement,
//      worldTransform(), returns fdk::Mat4d and does not convert to Matrix4.
//
//  v006 (kept): the engine reaches the axis input through the documented
//  GeomOpEngine accessor inputOpAt(node_input, frame), instead of a
//  (non-existent) engine-side geomOp(). All axis sampling happens inside the
//  engine -- no back-pointer to the owning op is needed. Knob values are read
//  with the engine's getValue<>(), exactly as the original USD version did.
//
//    inputOpAt(1, frame)   ->  the Op feeding input 1 at that frame
//                              (generated on demand; null if disconnected)
//    dynamic_cast<AxisOp*> ->  validate(true) + worldTransform() = world xform
//
//  The stage merge only pulls GeomOp inputs, so a non-GeomOp AxisOp on input 1
//  is ignored by the merge and read only here -- it can't corrupt the stage.
//
//  Node class: "StabilizeGeoUSD" (distinct from classic "StabilizeGeo").
// ============================================================================
#include "DDImage/ModifyGeomOp.h"
#include "DDImage/Knobs.h"
#include "DDImage/AxisOp.h"

#include "usg/geom/PointBasedPrim.h"

#include "StabilizeMath.h"   // shared, verified transform math (DD::Image::Matrix4)

#include <cmath>

using namespace DD::Image;

namespace {
  const char* const kModeNames[]  = { "stabilize", "matchmove", 0 };
  const char* const kOrderNames[] = { "ZXY", "XYZ", 0 };
}

class StabilizeGeoUSD : public ModifyGeomOp
{
public:
  const char* Class() const override { return "StabilizeGeoUSD"; }
  const char* node_help() const override {
    return "StabilizeGeoUSD (v007)\n\n"
           "USD-pipeline stabilize / match-move. Bakes mesh points by the motion of an "
           "Axis/Camera connected to input 1, on selected axes, with an optional reference "
           "frame.\n\n"
           "Connect geometry to input 0 and an Axis/Camera to the 'axis' input. "
           "mode: stabilize = cancel the axis motion; matchmove = re-apply it.\n\n"
           "Reference frame: when on, the chosen frame becomes the locked pose. At that "
           "frame the geo is unchanged; all other frames lock to it. stabilize then "
           "matchmove with the same settings returns the geo to normal.";
  }

  StabilizeGeoUSD(Node* node) : ModifyGeomOp(node, BuildEngine<Engine>()) {}

  // ---- inputs: 0 = geometry (base), 1 = axis/camera (optional) -----------
  int minimum_inputs() const override { return 2; }
  int maximum_inputs() const override { return 2; }

  Op* default_input(int i) const override {
    if (i == 1) return nullptr;                       // axis input is optional
    return ModifyGeomOp::default_input(i);
  }
  bool test_input(int i, Op* op) const override {
    if (i == 1) return dynamic_cast<AxisOp*>(op) != nullptr;
    return ModifyGeomOp::test_input(i, op);
  }
  const char* input_label(int i, char* buf) const override {
    return (i == 1) ? "axis" : ModifyGeomOp::input_label(i, buf);
  }

  // ---- knobs -------------------------------------------------------------
  void knobs(Knob_Callback f) override
  {
    ModifyGeomOp::knobs(f);

    int  mode = 0, order = stab::ROT_ZXY;
    bool tx = true, ty = true, tz = true;
    bool rx = true, ry = true, rz = true;
    bool sx = false, sy = false, sz = false;
    bool use_ref = false; double ref_frame = 1.0;

    Enumeration_knob(f, &mode,  kModeNames,  "mode", "mode");
    KnobModifiesAttribValues(f);
    Enumeration_knob(f, &order, kOrderNames, "rot_order", "rotation order");
    KnobModifiesAttribValues(f);

    Divider(f, "Axes to act on");
    Bool_knob(f, &tx, "translate_x", "translate x"); SetFlags(f, Knob::STARTLINE); KnobModifiesAttribValues(f);
    Bool_knob(f, &ty, "translate_y", "y"); KnobModifiesAttribValues(f);
    Bool_knob(f, &tz, "translate_z", "z"); KnobModifiesAttribValues(f);
    Bool_knob(f, &rx, "rotate_x", "rotate x"); SetFlags(f, Knob::STARTLINE); KnobModifiesAttribValues(f);
    Bool_knob(f, &ry, "rotate_y", "y"); KnobModifiesAttribValues(f);
    Bool_knob(f, &rz, "rotate_z", "z"); KnobModifiesAttribValues(f);
    Bool_knob(f, &sx, "scale_x", "scale x"); SetFlags(f, Knob::STARTLINE); KnobModifiesAttribValues(f);
    Bool_knob(f, &sy, "scale_y", "y"); KnobModifiesAttribValues(f);
    Bool_knob(f, &sz, "scale_z", "z"); KnobModifiesAttribValues(f);

    Divider(f, "Reference frame");
    Bool_knob(f, &use_ref, "use_reference", "use reference frame"); KnobModifiesAttribValues(f);
    Double_knob(f, &ref_frame, "ref_frame", "frame"); KnobModifiesAttribValues(f);
  }

  static Op* Build(Node* node) { return new StabilizeGeoUSD(node); }
  static const Description description;

private:
  class Engine : public ModifyEngine
  {
    PointBasedPrimFilterGeomEngineI _filter;
  public:
    // GeomOp::BuildEngine<Engine>() constructs us from whatever pointer type
    // the host NDK uses: Op* on Nuke 16.0, GeomOpNode* on Nuke 17.x. Taking it
    // as a template parameter keeps this source portable across both; the
    // ModifyEngine constructor performs the actual conversion.
    template <class ParentT>
    explicit Engine(ParentT* parent) : ModifyEngine(parent, &_filter) {}

    // World matrix of the axis on input 1 at an arbitrary frame (identity if
    // disconnected / not an axis). inputOpAt() is the GeomOpEngine accessor for
    // reaching an input Op at a specific frame.
    Matrix4 axisMatrixAt(double frame)
    {
      AxisOp* ax = dynamic_cast<AxisOp*>(inputOpAt(1, frame));
      if (!ax) return stab::ident();
      ax->validate(true);

      // NOTE: worldTransform() is the non-deprecated accessor, but it returns
      // the new-3D math type fdk::Mat4d, which has no implicit conversion to
      // DD::Image::Matrix4 -- and its storage order is not something to guess
      // at. matrix() returns exactly the Matrix4 that StabilizeMath.h expects
      // and holds the same value, so use it and silence the deprecation
      // warning locally rather than converting blind.
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      return ax->matrix();          // world transform at the requested frame
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic pop
#endif
    }

    void processPrim(usg::GeomSceneContext& context,
                     const usg::StageRef&   srcStage,
                     const usg::Path&       primPath) override
    {
      auto inPrim = usg::PointBasedPrim::getInStage(srcStage, primPath);
      if (!inPrim) return;
      auto outPrim = usg::PointBasedPrim::overrideInLayer(editLayer(), inPrim);
      if (!outPrim) return;
      const usg::Attribute pointsAttr = inPrim.getPointsAttr();
      if (!pointsAttr) return;

      // Knob values (none are animated).
      const bool                stabilize = (getValue<int>(0, "mode") == 0);
      const stab::RotationOrder order     = (stab::RotationOrder)getValue<int>(0, "rot_order");

      stab::AxisMask m;
      m.tx = getValue<bool>(true,  "translate_x"); m.ty = getValue<bool>(true,  "translate_y"); m.tz = getValue<bool>(true,  "translate_z");
      m.rx = getValue<bool>(true,  "rotate_x");    m.ry = getValue<bool>(true,  "rotate_y");    m.rz = getValue<bool>(true,  "rotate_z");
      m.sx = getValue<bool>(false, "scale_x");     m.sy = getValue<bool>(false, "scale_y");     m.sz = getValue<bool>(false, "scale_z");

      const bool   useRef   = getValue<bool>(false, "use_reference");
      const double refFrame = getValue<double>(1.0, "ref_frame");

      const Matrix4 refMotion = useRef ? axisMatrixAt(refFrame) : stab::ident();

      // Treat a connected axis as potentially animating -> write per-time.
      bool axisPresent = false;
      for (const auto& tt : context.processTimes()) {
        axisPresent = (dynamic_cast<AxisOp*>(inputOpAt(1, (double)tt)) != nullptr);
        break;
      }
      const bool isAnimating = pointsAttr.isAnimating() || axisPresent;

      usg::Vec3fArray points;
      for (const auto& time : context.processTimes()) {
        const Matrix4 motion = axisMatrixAt((double)time);
        const Matrix4 X = stab::buildResult(motion, stabilize, m, order, useRef, refMotion);

        if (inPrim.getPointsArray(points, time) > 0) {
          for (auto& p : points) {
            const float x = p.x, y = p.y, z = p.z;
            p.x = X.a00 * x + X.a01 * y + X.a02 * z + X.a03;
            p.y = X.a10 * x + X.a11 * y + X.a12 * z + X.a13;
            p.z = X.a20 * x + X.a21 * y + X.a22 * z + X.a23;
          }
          const fdk::TimeValue outTime = isAnimating ? time : fdk::defaultTimeValue();
          outPrim.setPoints(points, outTime);
          outPrim.setBoundsAttr(points, outTime);
        }
      }
    }
  };
};

const GeomOp::Description StabilizeGeoUSD::description("StabilizeGeoUSD", StabilizeGeoUSD::Build);
