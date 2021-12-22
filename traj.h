/*
 * Yifei Chen 2021/12/21
 * Fourier feature phase-based animation controller
 * 
 */

#ifndef TRAJ_H
#define TRAJ_H


#include <vector>
#include <set>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>
#include "delfem2/rig_geo3.h"
#include "delfem2/opengl/old/funcs.h"


class CParticle {
public:
    float pos[2] = { 0.f, 0.f };
    float velo[2] = { 0.f, 0.f };
    float color[3] = { 0.f, 0.f, 0.f };
    float mass = 1.f;
};


void draw_traj_line(std::vector<CParticle>& traj_points, std::vector<CParticle>& traj_points_sample)
{
    ::glLineWidth(2);
    ::glColor3d(0, 0, 0);
    ::glBegin(GL_LINE_STRIP);
    int jframe_total = traj_points.size();
    for (unsigned int jframe = 0; jframe < jframe_total - 1; ++jframe) {
        const double x0 = traj_points[jframe].pos[0];
        const double z0 = traj_points[jframe].pos[1];
        ::glVertex3d(x0, 0, z0);
    }
    ::glEnd();

    if (jframe_total > 1)
    {
        ::glColor3d(1, 0, 0);
        ::glBegin(GL_LINE_STRIP);
        const double x0 = traj_points[jframe_total - 2].pos[0];
        const double z0 = traj_points[jframe_total - 2].pos[1];
        const double dx1 = traj_points[jframe_total - 1].pos[0] - x0;
        const double dz1 = traj_points[jframe_total - 1].pos[1] - z0;
        ::glVertex3d(x0, 0, z0);
        ::glVertex3d(x0 + 10 * dx1, 0, z0 + 10 * dz1);
        ::glEnd();
    }

    for (auto& p : traj_points_sample)
    {
        ::glColor3d(1, 1, 1);
        ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, p.pos[0], 0, p.pos[1]);
    }


}


void draw(
    CParticle& ip)
{

    ::glColor3fv(ip.color);
    ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, ip.pos[0], 0, ip.pos[1]);
}


// -------------------------------------------


class damper_p {
public:
    float pos[2] = { 0.f, 0.f };
};


void draw_red_sphere(
    std::vector<float>& ip)
{
    // draw points
    float color[3] = { 0.9f,0.1f,0.1f };
    ::glColor3fv(color);
    ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, ip[0], 0, ip[1]);
}

void draw_blue_sphere(
    std::vector<float>& ip)
{
    // draw points
    float color[3] = { 0.1f,0.1f,0.9f };
    ::glColor3fv(color);
    ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, ip[0], 0, ip[1]);
}

void draw_black_sphere(
    std::vector<float>& ip)
{
    // draw points
    float color[3] = { 0.1f,0.1f,0.1f };
    ::glColor3fv(color);
    ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, ip[0], 0, ip[1]);
}

float lerp(float x, float y, float a)
{
    return (1.0f - a) * x + a * y;
}


float damper(float x, float g, float factor)
{
    return lerp(x, g, factor);
}

float z_target = 5.0f;

float halflife_to_damping(float halflife, float eps = 1e-5f)
{
    return (4.0f * 0.69314718056f) / (halflife + eps);
}

float fast_negexp(float x)
{
    return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
}

void spring_character_update(
    float& x,
    float& v,
    float& a,
    float v_goal,
    float halflife,
    float dt)
{
    float y = halflife_to_damping(halflife) / 2.0f;
    float j0 = v - v_goal;
    float j1 = a + j0 * y;
    float eydt = fast_negexp(y * dt);

    x = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
        (j1 / (y * y)) + j0 / y + v_goal * dt + x;
    v = eydt * (j0 + j1 * dt) + v_goal;
    a = eydt * (a - j1 * y * dt);
}

void spring_character_predict(
    float px[],
    float pv[],
    float pa[],
    int count,
    float x,
    float v,
    float a,
    float v_goal,
    float halflife,
    float dt)
{
    for (int i = 0; i < count; i++)
    {
        px[i] = x;
        pv[i] = v;
        pa[i] = a;
    }

    for (int i = 0; i < count; i++)
    {
        spring_character_update(px[i], pv[i], pa[i], v_goal, halflife, i * dt);
    }
}


#endif