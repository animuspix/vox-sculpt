#pragma once

#include "path.h"
#include "vmath.h"
#include "materials.h"

namespace tracing
{
    struct path_vt
    {
        path_vt(vmath::vec<3> _dir, vmath::vec<3> _ori, float _pdf, float _rho, float _rho_weight, float _power) :
            dir(_dir), ori(_ori), pdf(_pdf), rho_weight(_rho_weight), rho_sample(_rho), power(_power) {}

        vmath::vec<3> dir;
        vmath::vec<3> ori;
        float pdf = 1.0f; // Perspective-projected lens probabilities initialize at 100%; there's a proof somewhere, but naively
                          // we know that every sensor/film cell is sampled exactly once and every sensor/film cell *must* be sampled
                          // for a full image, so the relative probability for each sensor can never be less/greater than 1

        float rho_weight = 1.0f; // Totally dependant on the average scene SPD to be meaningful; initialize to 1 to hint that all colours
                                 // should be reflected by all rays by default

        float rho_sample = 0.5f; // Place path spectra near the white-point of our film response curve (see camera.h) by default
        float power = 1.0f; // Lights have unit energy by default (spikes up to light source wattage for final/starting verts)
        materials::instance* mat = nullptr; // Material at the intersection point, to allow recalculating shading as needed for light/camera path connections
    };

    class path
    {
    public:
        uint16_t front = 0;
        uint16_t size = 0;
        void push(path_vt vt);

        void resolve_path_weights(float* rho_out, float* pdf_out, float* response_out, float* power_out);

        void clear()
        {
            front = 0;
            size = 0;
        }

        static constexpr uint16_t capacity = 1734; // Our volume grid contains 1024x1024x1024 voxels; in the worst case, a volume with continuously-varying IOR will
                                                   // allow rays to walk through 1734 cells (top-right to bottom-left corner)
        path_vt vts[capacity];
    };
}
