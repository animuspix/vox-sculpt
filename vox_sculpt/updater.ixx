module;
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#undef min
#undef max

export module updater;

import geometry;
import camera;
import tracing;
import parallel;
import vox_ints;

//#define UPDATER_DBG
#ifdef UPDATER_DBG
#pragma optimize("", off)
#endif

export namespace render_updates
{
	void process()
	{
		// Definitions
		float xrot = 0.0f, yrot = 0.0f; // X, Y rotation (euler angles, we don't support roll atm)
		float z = 0.0f; // Zoom; delta locked at 0.1f (arbitrary)
		const float spinSpd = 360.0f / 30.0f; // Delta-angle locked to 12 degrees (360 degrees in a second @ 30fps)
		const float zSpd = 0.01f;

		// Volume updates
		xrot = (GetKeyState(VK_UP) & 0x8000) ? spinSpd :
			(GetKeyState(VK_DOWN) & 0x8000) ? -spinSpd : 0.0f;

		yrot = (GetKeyState(VK_LEFT) & 0x8000) ? spinSpd :
			(GetKeyState(VK_RIGHT) & 0x8000) ? -spinSpd : 0.0f;

		z = (GetKeyState('Z') & 0x8000) ? zSpd :
			(GetKeyState(VK_LCONTROL) & 0x8000) ? -zSpd : 0.0f;

		// Convert zoom/orbit to object transforms (scale & rotation), then send them over to our volume
		const bool spinning = xrot != 0.0f || yrot != 0.0f;
		const bool zooming = z != 0.0f;
		if (zooming || spinning)
		{
			if (spinning) geometry::spin(xrot, yrot);
			if (zooming) geometry::zoom(z);

			// Indicate transforms have changed, so we need to resample each render tile
			const u32 n_tiles = parallel::numTiles;
			for (u32 i = 0; i < n_tiles; i++)
			{
				tracing::views_resampling[i].store(1);
			}
		}
	}
}

#ifdef UPDATER_DBG
#pragma optimize("", on)
#endif