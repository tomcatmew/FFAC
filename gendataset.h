#ifndef FFCC_GENDATASET_H
#define FFCC_GENDATASET_H

#include <vector>
#include <set>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include "delfem2/rig_bvh.h"
#include "delfem2/rig_geo3.h"
#include "delfem2/opengl/old/funcs.h"

// q = qy*qxz
void SeparateYRot(
    double qy[4],
    double qxz[4],
    const double q[4]) {
  qxz[3] = std::sqrt(q[3] * q[3] + q[1] * q[1]);
  qy[0] = 0;
  qy[1] = q[1] / qxz[3];
  qy[2] = 0;
  qy[3] = q[3] / qxz[3];
  //
  double det = qy[3] * qy[3] + qy[1] * qy[1];
  double invdet = 1 / det;
  qxz[0] = (qy[3] * q[0] - qy[1] * q[2]) * invdet;
  qxz[1] = 0;
  qxz[2] = (qy[1] * q[0] + qy[3] * q[2]) * invdet;
  //
#ifndef NDEBUG
  assert(fabs(delfem2::Length_Quat(q) - 1.) < 1.0e-5);
  double qyqxz[4];
  delfem2::QuatQuat(qyqxz, qy, qxz);
  assert(fabs(q[0] - qyqxz[0]) < 1.0e-10);
  assert(fabs(q[1] - qyqxz[1]) < 1.0e-10);
  assert(fabs(q[2] - qyqxz[2]) < 1.0e-10);
  assert(fabs(q[3] - qyqxz[3]) < 1.0e-10);
#endif
}

void TransRotRoot(
    std::vector<delfem2::CVec2d> &aPosRoot2,
    std::vector<double> &aHeightRoot,
    std::vector<delfem2::CMat3d> &aRotRoot,
    std::vector<delfem2::CVec2d> &aRootOrientationZ,
    const std::vector<delfem2::CChannel_BioVisionHierarchy> &aChannelRotTransBone,
    const std::vector<double> &aValRotTransBone) {
  namespace dfm2 = delfem2;
  const unsigned int nch = aChannelRotTransBone.size();
  unsigned int nframe = aValRotTransBone.size() / nch;
  aPosRoot2.clear();
  aHeightRoot.clear();
  aRotRoot.clear();
  for (unsigned int iframe = 0; iframe < nframe; ++iframe) {
    aPosRoot2.emplace_back(aValRotTransBone[iframe * nch + 0], aValRotTransBone[iframe * nch + 2]);
    aHeightRoot.push_back(aValRotTransBone[iframe * nch + 1]);
    dfm2::CMat3d r = dfm2::CMat3d::Identity();
    for (unsigned int idim = 0; idim < 3; ++idim) {
      double a[3] = {0, 0, 0};
      const unsigned int iaxis = aChannelRotTransBone[3 + idim].iaxis;
      a[iaxis] = aValRotTransBone[iframe * nch + 3 + idim] * M_PI / 180.0;
      dfm2::CMat3d dr;
      dr.SetRotMatrix_Cartesian(a);
      r = r * dr; // this order is important
    }
    aRotRoot.emplace_back(r);
    //
    dfm2::CQuatd q;
    //r.GetQuat_RotMatrix(q.p);
    r.GetQuaternion();
    dfm2::CQuatd qy, qxy;
    SeparateYRot(qy.p, qxy.p, q.p);
    const double vi[3] = {0, 0, 1};
    double vo[3];
    dfm2::QuatVec(vo, qy.p, vi);
//    std::cout << iframe << " " << vo[0] << " " << vo[1] << " " << vo[2] << std::endl;
    aRootOrientationZ.emplace_back(vo[0], vo[2]);
  }
}

std::vector<unsigned int> GetIdBoneUsed(
    const std::vector<delfem2::CChannel_BioVisionHierarchy> &aChannelRotTransBone) {
  std::vector<unsigned int> aIdBoneUsed;
  std::set<unsigned int> setIdBoneUsed;
  for (const auto &ch : aChannelRotTransBone) {
    setIdBoneUsed.insert(ch.ibone);
  }
  setIdBoneUsed.erase(0);
  aIdBoneUsed.assign(setIdBoneUsed.begin(), setIdBoneUsed.end());
  return aIdBoneUsed;
}

void complete_quaternion(double q[4]) {
  double t = q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
  if (t > 1) {
    q[0] = q[0] / sqrt(t);
    q[1] = q[1] / sqrt(t);
    q[2] = q[2] / sqrt(t);
    q[3] = 0;
  } else {
    q[3] = sqrt(1 - t);
  }
}

// -------------------------------

/**
* nbones (number of bones), nframe (number of frames), nphase (number of phase), nsmpl_before/nsmpl_after (number of past/future trajectory sample points)
* @param vec_input : Pose Encoder input, trajectory + phase
* size of input = ( nsmpl_before + nsmpl_after ) * 2 + 2 * nphase
* @param vec_output : Pose Encoder output, 3 DoFs of root bone + 6 DoFs of other bones, will be converted to affine matrix
* size of output = 3 + 6 * nbones
* @param iframe : current frame number 
* @param aRootPos2 : trajectory position
* @param aRootDirZ : trajectory direction
* @param aIdBoneUsed : the index of bones that are used 

*/

class Encoder_Pose {
 public:
  static bool encode(
      std::vector<double> &vec_input,
      std::vector<double> &vec_output,
      int iframe,
      const std::vector<delfem2::CVec2d> &aRootPos2,
      const std::vector<delfem2::CVec2d> &aRootDirZ,
      const std::vector<double> &aPhase,
      const std::vector<delfem2::CRigBone> &aBone,
      const std::vector<unsigned int> &aIdBoneUsed,
      const std::vector<delfem2::CChannel_BioVisionHierarchy> &aChannelRotTransBone,
      const std::vector<double> &aValRotTransBone) {
    namespace dfm2 = delfem2;
    std::vector<delfem2::CRigBone> aBone0 = aBone;
    const unsigned int nframe = aRootPos2.size();
    const dfm2::CVec2d p0 = aRootPos2[iframe];
    const dfm2::CVec2d dz0 = aRootDirZ[iframe];  // local z coord
    assert(fabs(dz0.norm() - 1) < 1.0e-5);
    const dfm2::CVec2d dx0(dz0.y, -dz0.x);  // local x coord
    if (aPhase[iframe] < -0.5) { return false; }
    dfm2::CQuatd q0y;
    {  // y-axis rotation of body
      const dfm2::CMat3d R0(
          dx0.x, 0, dz0.x,
          0, 1, 0,
          dx0.y, 0, dz0.y);
      //R0.GetQuat_RotMatrix(q0y.p);
      R0.GetQuaternion();
      q0y.SetSmallerRotation();
    }
    // ------------------
    vec_input.clear();
    vec_input.push_back(aPhase[iframe]);  // phase
    // 2D trajectory points relative to body
    for (int ismpl = -nsmpl_before; ismpl <= nsmpl_after; ismpl++) {
      if (ismpl == 0) { continue; }
      const int jframe = iframe + ismpl * smpl_nframe;
      if (jframe < 0 || jframe >= nframe) { return false; }
      vec_input.push_back(dx0.dot(aRootPos2[jframe] - p0));  // relative position
      vec_input.push_back(dz0.dot(aRootPos2[jframe] - p0));  // relative position
    }
    assert(vec_input.size() == nDimIn);
    // above: input
    // ------------------
    // below: output
    vec_output.clear();
    dfm2::SetPose_BioVisionHierarchy(
        aBone0,
        aChannelRotTransBone,
        aValRotTransBone.data() + iframe * aChannelRotTransBone.size());
    { // rootbone
      // get height
      const dfm2::CRigBone &b = aBone0[0];
      { // put height
        const double y0 = b.affmat3Global[1 * 4 + 3];
        vec_output.push_back(y0 * 0.05);  // 1: y height
        assert(fabs(p0.x - b.affmat3Global[0 * 4 + 3]) < 1.0e-5);
        assert(fabs(p0.y - b.affmat3Global[2 * 4 + 3]) < 1.0e-5);
      }
      // get xz rotations
      dfm2::CMat3d R0(
          b.affmat3Global[0 * 4 + 0],
          b.affmat3Global[0 * 4 + 1],
          b.affmat3Global[0 * 4 + 2],
          b.affmat3Global[1 * 4 + 0],
          b.affmat3Global[1 * 4 + 1],
          b.affmat3Global[1 * 4 + 2],
          b.affmat3Global[2 * 4 + 0],
          b.affmat3Global[2 * 4 + 1],
          b.affmat3Global[2 * 4 + 2]);
      dfm2::CQuatd q0;
      //R0.GetQuat_RotMatrix(q0.p);
      R0.GetQuaternion();
      dfm2::CQuatd q0yTmp, q0xz;
      SeparateYRot(q0yTmp.p, q0xz.p, q0.p);  // q0 = q0y*q0xy
      q0yTmp.SetSmallerRotation();
      q0xz.SetSmallerRotation();
      assert((q0y - q0yTmp).norm() < 1.0e-5);
      vec_output.push_back(q0xz.x);  // xrot
      vec_output.push_back(q0xz.z);  // zrot
#ifndef NDEBUG
      { // z dir
        double vo[3], vi[3] = {0, 0, 1};
        dfm2::QuatVec(vo, q0y.p, vi);
        assert(fabs(vo[0] - dz0.x) < 1.0e-5);
        assert(fabs(vo[2] - dz0.y) < 1.0e-5);
      }
      { // x dir
        double vo[3], vi[3] = {1, 0, 0};
        dfm2::QuatVec(vo, q0y.p, vi);
        assert(fabs(vo[0] - dx0.x) < 1.0e-5);
        assert(fabs(vo[2] - dx0.y) < 1.0e-5);
      }
#endif
    }
    for (auto ib : aIdBoneUsed) {
      const dfm2::CRigBone &b = aBone0[ib];
      const dfm2::CVec3d pi(
          b.affmat3Global[0 * 4 + 3],
          b.affmat3Global[1 * 4 + 3],
          b.affmat3Global[2 * 4 + 3]);
      const dfm2::CMat3d Ri(
          b.affmat3Global[0 * 4 + 0],
          b.affmat3Global[0 * 4 + 1],
          b.affmat3Global[0 * 4 + 2],
          b.affmat3Global[1 * 4 + 0],
          b.affmat3Global[1 * 4 + 1],
          b.affmat3Global[1 * 4 + 2],
          b.affmat3Global[2 * 4 + 0],
          b.affmat3Global[2 * 4 + 1],
          b.affmat3Global[2 * 4 + 2]);
      dfm2::CQuatd qi;
      //Ri.GetQuat_RotMatrix(qi.p);
      Ri.GetQuaternion();
      qi.SetSmallerRotation();
      assert(fabs(qi.norm() - 1.0) < 1.0e-5);
      dfm2::CQuatd q0yinv = q0y.conjugate();
      const double pi1[3] = {pi.x - p0.x, pi.y, pi.z - p0.y};
      double pi2[3];
      dfm2::QuatVec(pi2, q0yinv.p, pi1);
      vec_output.push_back(pi2[0] * 0.05);
      vec_output.push_back(pi2[1] * 0.05);
      vec_output.push_back(pi2[2] * 0.05);
      dfm2::CQuatd q0yinvqi = q0yinv * qi;
      q0yinvqi.SetSmallerRotation();
      vec_output.push_back(q0yinvqi.x);
      vec_output.push_back(q0yinvqi.y);
      vec_output.push_back(q0yinvqi.z);
    }
    assert(vec_output.size() == nDimOut);
    return true;
  }

  /**
   * Apply the rotation and transformation to bones
   * @param aBone : vector of CrigBones
   * @param aOut : output of pose predicton network, 3 Dofs root bone and 6 Dofs left bones 
   * @param aIdBoneUsed : the index of bones that are used 
   * @param q0y : orientation of the trajectory
   * @param p0 : current global position of root
   */
  static void decode(
      std::vector<delfem2::CRigBone> &aBone,
      const std::vector<double> &aOut,
      const std::vector<unsigned int> &aIdBoneUsed,
      const delfem2::CQuatd &q0y,
      const delfem2::CVec2d &p0) {
    namespace dfm2 = delfem2;
    assert(aOut.size() == nDimOut);
//    const double dphase = (aOut[0] + 1) * 0.01;
    aBone[0].affmat3Global[0 * 4 + 3] = p0.x;
    aBone[0].affmat3Global[1 * 4 + 3] = aOut[0] * 20;
    aBone[0].affmat3Global[2 * 4 + 3] = p0.y;
    {  // rotation of bone_0
      double q0xy[4] = {aOut[1], 0, aOut[2], 0.0};  // x,y,z,w
      complete_quaternion(q0xy);
      double q0[4];
      dfm2::QuatQuat(q0, q0y.p, q0xy);
      double R0[9];
      dfm2::Mat3_Quat(R0, q0);
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          aBone[0].affmat3Global[i * 4 + j] = R0[i * 3 + j];
        }
      }
    }
    // rotation and translation of bone_i
    for (int iib = 0; iib < aIdBoneUsed.size(); ++iib) {
      const int ib0 = aIdBoneUsed[iib];
      dfm2::CRigBone &b = aBone[ib0];
      const double pi2[3] = {
          aOut[3 + iib * 6 + 0] * 20,
          aOut[3 + iib * 6 + 1] * 20,
          aOut[3 + iib * 6 + 2] * 20};
      double pi1[3];
      dfm2::QuatVec(pi1, q0y.p, pi2);
      b.affmat3Global[0 * 4 + 3] = pi1[0] + p0.x;  //pi1[0]+p0.x;
      b.affmat3Global[1 * 4 + 3] = pi1[1];  //pi1[1];
      b.affmat3Global[2 * 4 + 3] = pi1[2] + p0.y;  //pi1[2]+p0.y;
      //
      double q0yinvqi[4] = {
          aOut[3 + iib * 6 + 3],
          aOut[3 + iib * 6 + 4],
          aOut[3 + iib * 6 + 5],
          0.0};
      complete_quaternion(q0yinvqi);
      double qi[4];
      dfm2::QuatQuat(qi, q0y.p, q0yinvqi);
      double Ri[9];
      dfm2::Mat3_Quat(Ri, qi);
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          b.affmat3Global[i * 4 + j] = Ri[i * 3 + j];
        }
      }
      //
      b.affmat3Global[3 * 4 + 0] = 0;
      b.affmat3Global[3 * 4 + 1] = 0;
      b.affmat3Global[3 * 4 + 2] = 0;
      b.affmat3Global[3 * 4 + 3] = 1;
    }
  }

  /**
   * draw input data, draw the rotated trajectory 
   * @param[in] aIn : relative positions to next frame's trajectory
   * @param[in] q0y : orientation of the trajectory
   * @param[in] p0 : current global position of root
   */
  static void draw_in(
      const std::vector<double> &aIn,
      const delfem2::CQuatd &q0y,
      const delfem2::CVec2d &p0) {
    namespace dfm2 = delfem2;
    assert(aIn.size() == nDimIn);
    ::glDisable(GL_LIGHTING);
//    double phase = aIn[0];
    int inc = 0;
    for (int ismpl = -nsmpl_before; ismpl <= nsmpl_after; ++ismpl) {
      if (ismpl == 0) { continue; }
      ::glColor3d(0, 0, 0);
      ::glLineWidth(3);
      dfm2::CVec3d dp0(aIn[1 + inc * 2 + 0], 0.0, aIn[1 + inc * 2 + 1]);  // position
      dfm2::CMat3d R0 = dfm2::CMat3d::Quat(q0y.p);
      dfm2::CVec3d q0 = dfm2::CVec3d(p0.x, 0, p0.y) + R0 * dp0;
      dfm2::opengl::DrawSphereAt(32, 32, 0.5, q0.x, q0.y, q0.z);
      inc++;
    }
    {  // data at origin
      const double z0[3] = {0., 0., 1.};
      double z1[3];
      dfm2::QuatVec(z1, q0y.p, z0);
      ::glColor3d(1, 0, 0);
      ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, p0.x, 0, p0.y);
      ::glDisable(GL_LIGHTING);
      ::glLineWidth(3);
      ::glBegin(GL_LINES);
      ::glVertex3d(p0.x, 0., p0.y);
      ::glVertex3d(
          2*z1[0]+p0.x,
          2*z1[1],
          2*z1[2]+p0.y);
      ::glEnd();
    }
  }
 public:
  static constexpr int nsmpl_before = 5;
  static constexpr int nsmpl_after = 5;
  static constexpr int smpl_nframe = 20;
  static constexpr int nDimIn = 1 + 2 * (nsmpl_before + nsmpl_after); // phase and trajectory points
  static constexpr int nDimOut = 3 + 30 * 6;
};


// -------------------------------
/**
*  @param aIn : input trajectory, vector of 2D positions 
*  input size : ( nsmpl_before + nsmpl_after ) * 2
*  @param aOut : output orientation + phase
*  output size : 2 [root.x, root.y] + 2 [sin(2*pi*N),cos(2*pi*N)] = 4
* 
*/

class Encoder_Phase {
 public:
  static bool encode(
      std::vector<double> &aIn,
      int iframe,
      const std::vector<delfem2::CVec2d> &aRootPos2) {
    namespace dfm2 = delfem2;
    const unsigned int nframe = aRootPos2.size();
    // ------------------
    aIn.clear();
    bool is_input_ok = true;
    for (int ismpl = -nsmpl_before; ismpl <= nsmpl_after; ismpl++) {
      if (ismpl == 0) { continue; }
      const int jframe = iframe + ismpl * smpl_nframe;
      if (jframe < 0 || jframe >= nframe) {
        aIn.push_back(0.);
        aIn.push_back(0.);
        is_input_ok = false;
      } else {
        dfm2::CVec2d p01 = aRootPos2[jframe] - aRootPos2[iframe];
        aIn.push_back(p01.x);
        aIn.push_back(p01.y);
      }
    }
    assert(aIn.size() == nDimIn);
    return is_input_ok;
  }

  static void encode(
      std::vector<double> &aIn,
      std::vector<double> &aOut,
      int iframe,
      const std::vector<delfem2::CVec2d> &aRootPos2,
      const std::vector<delfem2::CVec2d> &aRootDirZ,
      const std::vector<double> &aPhase) {
    namespace dfm2 = delfem2;
    const unsigned int nframe = aRootPos2.size();
    const bool is_input_ok = encode(
        aIn,
        iframe, aRootPos2);
    // ------------------
    // below: output
    aOut.clear();
    { // phase sine & cosine
      const double phase0 = aPhase[iframe];
      if (phase0 < -0.5 || !is_input_ok) {
        aOut.push_back(0.);
        aOut.push_back(0.);
      } else {
        aOut.push_back(std::sin(2 * M_PI * phase0));
        aOut.push_back(std::cos(2 * M_PI * phase0));
      }
    }
    {  // 2D points relative to body
      aOut.push_back(aRootDirZ[iframe].x);
      aOut.push_back(aRootDirZ[iframe].y);
    }
    assert(aOut.size() == nDimOut);
  }

  static void decode(
      std::vector<double> &aPhase,
      std::vector<delfem2::CVec2d> &aRootDirZ,
      int iframe,
      std::vector<double> &aOut,
      const std::vector<delfem2::CVec2d> &aRootPos2) {
    namespace dfm2 = delfem2;
    assert(aOut.size() == nDimOut);
    if (aPhase.size() < iframe + 1) { aPhase.resize(iframe + 1); }
    if (aRootDirZ.size() < iframe + 1) { aRootDirZ.resize(iframe + 1); }
    {
      double s0 = aOut[0];
      double c0 = aOut[1];
      if (s0 * s0 + c0 * c0 < 1.0e-3) {
        aPhase[iframe] = -1;
      } else {
        double radian = atan2(s0, c0);
        double phase0 = radian * 0.5 / M_PI;
        if (phase0 < 0) { phase0 += 1.; }
        assert(phase0 >= 0 && phase0 <= 1.0);
        aPhase[iframe] = phase0;
      }
    }
    {
      double x0 = aOut[2];
      double z0 = aOut[3];
      dfm2::CVec2d dir(x0, z0);
      //std::cout << "### " << iframe << " " << dir << std::endl;
      dir.normalize();
      aRootDirZ[iframe] = dir;
    }
  }

  /*
  * draw the trajectory
  * @param aIn : relative past/future trajectory points positions
  * @param p0 : current root position 
  */
  static void draw_in(
      const std::vector<double> &aIn,
      const delfem2::CVec3d &p0) {
    namespace dfm2 = delfem2;
    assert(aIn.size() == nDimIn);
    ::glDisable(GL_LIGHTING);
    int inc = 0;
    for (int ismpl = -nsmpl_before; ismpl <= nsmpl_after; ++ismpl) {
      if (ismpl == 0) { continue; }
      ::glColor3d(0, 0, 0);
      ::glLineWidth(3);
      dfm2::CVec3d dp0(aIn[inc * 2 + 0], 0.0, aIn[inc * 2 + 1]);  // position
      const dfm2::CVec3d q0 = dfm2::CVec3d(p0.x, 0, p0.y) + dp0;
      dfm2::opengl::DrawSphereAt(32, 32, 0.5, q0.x, q0.y, q0.z);
      inc++;
    }
  }
 public:
  static constexpr int nsmpl_before = 5;
  static constexpr int nsmpl_after = 5;
  static constexpr int smpl_nframe = 20;

  // positions (local x,z) and phase
  static constexpr int nDimIn = 2 * (nsmpl_before + nsmpl_after);

  // phase (sin, cos) and orientation (local x,z)
  static constexpr int nDimOut = 4;
};

#endif //INC_1_VISPHASE_GENDATASET_H
