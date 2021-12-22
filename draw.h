#ifndef FFCC_DRAW_H
#define FFCC_DRAW_H

#include "delfem2/color.h"
#include "delfem2/opengl/old/funcs.h"
#include "delfem2/opengl/old/rigv3.h"
#include <GLFW/glfw3.h>

void DrawPhase(
    double phase,
    double asp) {
  ::glMatrixMode(GL_PROJECTION);
  ::glPushMatrix();
  ::glLoadIdentity();
  ::glOrtho(-asp,+asp, -1,+1, -1,+1);
  ::glMatrixMode(GL_MODELVIEW);
  ::glPushMatrix();
  ::glLoadIdentity();

  const double cent[2] = {-asp+0.25, 1.0-0.25};
  const double rad = 0.2;

  ::glDisable(GL_LIGHTING);
  ::glLineWidth(2);
  ::glColor3d(0,0,0);
  if( phase > -0.5 ) {
    ::glBegin(GL_LINES);
    ::glVertex2d(cent[0], cent[1]);
    ::glVertex2d(cent[0] + rad * cos(phase * 2 * M_PI), cent[1] + rad * sin(phase * 2 * M_PI));
    ::glEnd();
  }

  delfem2::opengl::DrawCircleWire(
      delfem2::CVec3d(0,0,1),
      delfem2::CVec3d(cent[0], cent[1], 0),
      rad);

  ::glMatrixMode(GL_PROJECTION);
  ::glPopMatrix();
  ::glMatrixMode(GL_MODELVIEW);
  ::glPopMatrix();
}

void draw_legend(
    double asp) {
  ::glMatrixMode(GL_PROJECTION);
  ::glPushMatrix();
  ::glLoadIdentity();
  ::glOrtho(-asp,+asp, -1,+1, -1,+1);
  ::glMatrixMode(GL_MODELVIEW);
  ::glPushMatrix();
  ::glLoadIdentity();

  {
    int N = 20;
    ::glBegin(GL_QUADS);
    for(unsigned int i=0;i<N;++i) {
      double r0, g0, b0;
      delfem2::GetRGB_HSV(
          r0, g0, b0,
          (1./N)*(i+0), 1., 0.8);
      double r1, g1, b1;
      delfem2::GetRGB_HSV(
          r1, g1, b1,
          (1./N)*(i+1), 1., 0.8);
      ::glColor3d(r0, g0, b0);
      ::glVertex2d(-0.5+(1.0/N)*(i+0), 0.7);
      ::glVertex2d(-0.5+(1.0/N)*(i+0), 0.9);
      ::glColor3d(r1, g1, b1);
      ::glVertex2d(-0.5+(1.0/N)*(i+1), 0.9);
      ::glVertex2d(-0.5+(1.0/N)*(i+1), 0.7);
    }
    ::glEnd();
  }

  ::glMatrixMode(GL_PROJECTION);
  ::glPopMatrix();
  ::glMatrixMode(GL_MODELVIEW);
  ::glPopMatrix();
}


void DrawTrajectories(
    unsigned int iframe,
    int nsmpl_before,
    int nsmpl_after,
    unsigned int smpl_nframe,
    const std::vector<delfem2::CVec2d> &aRootPos2,
    const std::vector<delfem2::CVec2d> &aRootOrientationZ ) {
  namespace dfm2 = delfem2;
  const unsigned int nframe = aRootPos2.size();
  {
    unsigned int jframe0 = std::max(int(iframe - 1.3 * nsmpl_before * smpl_nframe), 0);
    unsigned int jframe1 = std::min(int(iframe + 1.3 * nsmpl_after * smpl_nframe), int(nframe - 1));
    ::glLineWidth(1);
    ::glColor3d(0, 0, 0);
    ::glBegin(GL_LINE_STRIP);
    for (unsigned int jframe = jframe0; jframe < jframe1; ++jframe) {
      const double x0 = aRootPos2[jframe].x;
      const double z0 = aRootPos2[jframe].y;
      ::glVertex3d(x0, 0, z0);
    }
    ::glEnd();
  }
  //
  for (int ismpl = -nsmpl_before; ismpl <= nsmpl_after; ++ismpl) {
    int jframe = iframe + ismpl * smpl_nframe;
    if (jframe < 0 || jframe >= nframe) { continue; }
    if (ismpl == 0) { ::glColor3d(1, 0, 0); }
    else { ::glColor3d(0, 0, 0); }
    dfm2::CVec2d p0 = aRootPos2[jframe];
    ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, p0.x, 0, p0.y);
  }
  {
    ::glColor3d(1, 0, 0);
    ::glDisable(GL_LIGHTING);
    ::glLineWidth(3);
    ::glBegin(GL_LINES);
    dfm2::CVec2d p0 = aRootPos2[iframe];
    dfm2::CVec2d p1 = p0 + 2 * aRootOrientationZ[iframe];
    ::glVertex3d(p0.x, 0, p0.y);
    ::glVertex3d(p1.x, 0, p1.y);
    ::glEnd();
  }
}

class Floor{
 public:
  // floor
  void draw_geometry() const {
    ::glDisable(GL_LIGHTING);
    ::glBegin(GL_QUADS);
    ::glNormal3d(0, 1, 0);
    ::glVertex3d(-wfloor, yfloor, -wfloor);
    ::glVertex3d(+wfloor, yfloor, -wfloor);
    ::glVertex3d(+wfloor, yfloor, +wfloor);
    ::glVertex3d(-wfloor, yfloor, +wfloor);
    ::glEnd();
  }
  void draw_checkerboard() const {
    constexpr int ndiv = 25;
    const double len = wfloor*2.0/ndiv;
    ::glDisable(GL_LIGHTING);
    ::glBegin(GL_QUADS);
    ::glNormal3d(0, 1, 0);
    for(unsigned int idiv=0;idiv<ndiv;++idiv) {
      for (unsigned int jdiv = 0; jdiv < ndiv; ++jdiv) {
        if( (idiv + jdiv) % 2 == 0 ){
          ::glColor4f(0.6f, 0.6f, 0.5f, 1.0f);
        }
        else{
          ::glColor4f(0.55f, 0.55f, 0.45f, 1.0f);
        }
        const double x0 = -wfloor + len*(idiv+0);
        const double x1 = -wfloor + len*(idiv+1);
        const double z0 = -wfloor + len*(jdiv+0);
        const double z1 = -wfloor + len*(jdiv+1);
        ::glVertex3d(x0, yfloor, z0);
        ::glVertex3d(x1, yfloor, z0);
        ::glVertex3d(x1, yfloor, z1);
        ::glVertex3d(x0, yfloor, z1);
      }
    }
    ::glEnd();
  }
 public:
  double wfloor, yfloor;
};

void DrawFloorShadow(
    const std::vector<delfem2::CRigBone> &vec_rig_bone,
    const Floor& floor,
    float yfloor) {
  namespace lcl = delfem2::opengl::old::funcs;
  GLboolean is_lighting = ::glIsEnabled(GL_LIGHTING);
  GLboolean is_blend = ::glIsEnabled(GL_BLEND);
  {  // draw floor
    ::glClearStencil(0);
    { // draw floor (stencil 1)
      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_ALWAYS, 1, static_cast<GLuint>(~0));
      glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
      floor.draw_checkerboard();
    }
    { // draw stensil
      glColorMask(0, 0, 0, 0);
      glDepthMask(0);
      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_EQUAL, 1, static_cast<GLuint>(~0));
      glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
      glPushMatrix();
      {
        float plane[4] = {0, 1, 0, -yfloor - 0.001f};
        float lpos[4] = {0, 100, 0, 1};
        float m_shadow[16];
        lcl::ShadowMatrix(m_shadow, plane, lpos);
        glMultMatrixf(m_shadow);
      }
      delfem2::opengl::DrawBone_Octahedron(
          vec_rig_bone,
          5, -1,
          0.1, 1.0);
      glPopMatrix();
      glColorMask(1, 1, 1, 1);
      glDepthMask(1);
    }
    { // draw shadow
      glStencilFunc(GL_EQUAL, 2, static_cast<GLuint>(~0));
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      ::glDisable(GL_DEPTH_TEST);
      ::glDisable(GL_LIGHTING);
      glColor4f(0.1f, 0.1f, 0.1f, 0.5f);
      floor.draw_geometry();
      glEnable(GL_DEPTH_TEST);
      glDisable(GL_BLEND);
      glDisable(GL_STENCIL_TEST);
    }
  }
  if (is_lighting) { ::glEnable(GL_LIGHTING); }
  if (is_blend) { ::glEnable(GL_BLEND); }
}

void draw_trajectories(
    unsigned int iframe,
    int nsmpl_before,
    int nsmpl_after,
    unsigned int smpl_nframe,
    const std::vector<delfem2::CVec2d> &vec_pos2,
    const std::vector<delfem2::CVec2d> &vec_dirz,
    const std::vector<double> &vec_phase ) {
  namespace dfm2 = delfem2;
  const unsigned int nframe = vec_pos2.size();
  {  // thin line
    unsigned int jframe0 = std::max(int(iframe - 1.3 * nsmpl_before * smpl_nframe), 0);
    unsigned int jframe1 = std::min(int(iframe + 1.3 * nsmpl_after * smpl_nframe), int(nframe - 1));
    ::glLineWidth(1);
    ::glColor3d(0, 0, 0);
    ::glBegin(GL_LINE_STRIP);
    for (unsigned int jframe = jframe0; jframe < jframe1; ++jframe) {
      const double x0 = vec_pos2[jframe].x;
      const double z0 = vec_pos2[jframe].y;
      ::glVertex3d(x0, 0, z0);
    }
    ::glEnd();
  }
  //
  for (int ismpl = -nsmpl_before; ismpl <= nsmpl_after; ++ismpl) {
    int jframe = iframe + ismpl * smpl_nframe;
    if (jframe < 0 || jframe >= nframe) { continue; }
    dfm2::CVec2d p0 = vec_pos2[jframe];
    const double ph0 = vec_phase[jframe];
    {
      double r, g, b;
      dfm2::GetRGB_HSV(
          r, g, b,
          ph0, 1., 0.8);
      ::glColor3d(r,g,b);
    }
    ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, p0.x, 0, p0.y);
    ::glDisable(GL_LIGHTING);
    ::glLineWidth(3);
    ::glBegin(GL_LINES);
    dfm2::CVec2d p1 = p0 + 2 * vec_dirz[jframe];
    ::glVertex3d(p0.x, 0, p0.y);
    ::glVertex3d(p1.x, 0, p1.y);
    ::glEnd();
  }
}

#endif
