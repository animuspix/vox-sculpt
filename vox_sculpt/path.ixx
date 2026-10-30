module;
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#undef min
#undef max

export module path;

import vmath;
import materials;
import vox_ints;

export namespace tracing
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
		u16 front = 0;
		u16 size = 0;
		void push(path_vt vt)
		{
			if (front >= capacity) DebugBreak();
			vts[front] = vt;
			front++;
			size++;
		}

		void resolve_path_weights(float* rho_out, float* pdf_out, float* response_out, float* power_out)
		{
			float pdf = 1.0f;
			float response = 1.0f;
			float power = 1.0f;
			*rho_out = vts[0].rho_sample;
			for (u32 i = 0; i < size; i++)
			{
				auto& bounce = vts[i];
				pdf *= bounce.pdf;
				response *= bounce.rho_weight;
				power *= bounce.power;

				// Process angular energy loss between vertices (the last cos(theta) in the rendering equation)
				if (i < static_cast<u32>(size - 1)) power *= vts[i].dir.dot(vts[i + 1].dir);
			}
			*pdf_out = pdf;
			*response_out = response;
			*power_out = power;

			// Force assets to render in solid colour for debugging
			//#define HIGHLIGHT_SURFACES
#ifdef HIGHLIGHT_SURFACES
			if (vts[0].mat != nullptr)
			{
				*response_out = 1.0f;
				*rho_out = 0.85f;
			}
#endif
		}

		void clear()
		{
			front = 0;
			size = 0;
		}
		static constexpr u16 capacity = 1734; // Our volume grid contains 1024x1024x1024 voxels; in the worst case, a volume with continuously-varying IOR will
		// allow rays to walk through 1734 cells (top-right to bottom-left corner)
		path_vt vts[capacity];
	};
}
