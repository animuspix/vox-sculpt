module;
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#undef min
#undef max

export module geometry;

#pragma once

import vmath;
import materials;
import mem;
import parallel;
import spectra;
import vox_ints;
import <cmath>;
import <chrono>;
import <cstdio>;
import <atomic>;

//#define GEOMETRY_DBG
#ifdef GEOMETRY_DBG
#pragma optimize("", off)
#endif

namespace geometry
{
	export struct vol
	{
		static constexpr u32 width = 1024; // All volumes are 1024 * 1024 * 1024
		static constexpr u32 slice_area = width * width;
		static constexpr u32 res = slice_area * width;
		static constexpr float cell_size = 1.0f / width;
		static constexpr u32 max_cell_ndx_per_axis = width - 1;
		struct transform_nfo
		{
			vmath::vec<3> scale; // Bounding-box scale on x/y/z
			vmath::vec<3> pos; // World-space position
			vmath::vec<4> orientation; // World-space orientation, specified as a quaternion
			vmath::vec<2> ss_v0; // Top-left of the screen-space quad projected from the volume's bounding-box, updated on camera transform
			vmath::vec<2> ss_v1; // Top-right of the screen-space bounding quad
			vmath::vec<2> ss_v2; // Bottom-left of the screen-space bounding quad
			vmath::vec<2> ss_v3; // Bottom-right of the screen-space bounding quad
		};

		// Volume metadata (material, transform information)
		struct vol_nfo
		{
			materials::instance mat;
			transform_nfo transf;
		};
		static vol_nfo* metadata;

		// Our geometry is composed of individual bits, grouped into 64-bit chunks;
		// metachunks take that abstraction one layer higher by providing groups of 2x2x2
		// chunks for efficient traversal (one metachunk is one cacheline)
		// Individual voxels (within chunks) occupy one bit each; order is left-right/front-back/top-bottom
		// front view:
		// 00 01 02 03
		// 16 17 18 19
		// 32 33 34 35
		// 48 49 50 51
		// top view:
		// 00 01 02 03
		// 04 05 06 07
		// 08 09 10 11
		// 12 13 14 15
		// bottom view:
		// 48 49 50 51
		// 52 53 54 55
		// 56 57 58 59
		// 60 61 62 63
		// We evaluate these cells by constructing a 64-bit mask for the one we want to test; if the current chunk AND the mask is nonzero, we have a set cell, otherwise we have an empty one
		static u8* metachunk_occupancies; // Direct mask of occupancies per-metachunk, for faster testing during chunk/metachunk traversal
		struct metachunk
		{
			// Metachunk dimensions in chunks
			static constexpr u32 res_x = 2;
			static constexpr u32 res_y = 2;
			static constexpr u32 res_z = 2;
			static constexpr u32 res_xy = res_x * res_y;
			static constexpr u32 res = res_xy * res_z;

			// Chunk dimensions in voxels
			static constexpr u32 chunk_res_x = 4;
			static constexpr u32 chunk_res_y = 4;
			static constexpr u32 chunk_res_z = 4;
			static constexpr u32 chunk_res_xy = chunk_res_x * chunk_res_y;

			// Metachunk dimensions in voxels
			static constexpr u32 num_vox_x = res_x * chunk_res_x;
			static constexpr u32 num_vox_y = res_y * chunk_res_y;
			static constexpr u32 num_vox_z = res_z * chunk_res_z;
			static constexpr u32 num_vox_xy = num_vox_x * num_vox_y;
			static constexpr u32 num_vox = num_vox_xy * num_vox_z;

			// Metachunk data
			u64 chunks[res];
		};
		static constexpr u32 num_metachunks_x = width / metachunk::num_vox_x;
		static constexpr u32 num_metachunks_y = width / metachunk::num_vox_y;
		static constexpr u32 num_metachunks_z = width / metachunk::num_vox_z;
		static constexpr u32 num_metachunks_xy = num_metachunks_x * num_metachunks_y;
		static constexpr u32 num_metachunks = num_metachunks_xy * num_metachunks_z;
		static metachunk* metachunks;
		static u32 chunk_index_solver(vmath::vec<3, i32> uvw_floored) // Returns chunk index
		{
			// Scalarized logic to reduce vec<n> constructor calls
			//////////////////////////////////////////////////////

			const i32 chunk_x = (uvw_floored.x() % metachunk::num_vox_x) / metachunk::chunk_res_x;
			const i32 chunk_y = (uvw_floored.y() % metachunk::num_vox_y) / metachunk::chunk_res_y;
			const i32 chunk_z = (uvw_floored.z() % metachunk::num_vox_z) / metachunk::chunk_res_z;

			// Compose returnable index
			return static_cast<u32>(chunk_x + // Local scanline offset
				(chunk_y * metachunk::res_x) + // Local slice offset
				(chunk_z * metachunk::res_xy)); // Volume offset;
		}
		static u32 metachunk_index_solver(vmath::vec<3, i32> uvw_floored) // Returns metachunk index
		{
			// Scalarized logic to reduce vec<n> constructor calls
			//////////////////////////////////////////////////////

			const u32 metachunk_x = uvw_floored.x() / metachunk::num_vox_x;
			const u32 metachunk_y = uvw_floored.y() / metachunk::num_vox_y;
			const u32 metachunk_z = uvw_floored.z() / metachunk::num_vox_z;
			return static_cast<u32>(metachunk_x + // Local scanline offset
				(metachunk_y * num_metachunks_x) + // Local slice offset
				(metachunk_z * num_metachunks_xy)); // Volume offset;
		}
		static u32 metachunk_index_solver_fast(vmath::vec<3, i32> metachunk_uvw) // Returns metachunk index
		{
			return static_cast<u32>(metachunk_uvw.x() + // Local scanline offset
				(metachunk_uvw.y() * num_metachunks_x) + // Local slice offset
				(metachunk_uvw.z() * num_metachunks_xy)); // Volume offset;
		}
		struct voxel_ndces
		{
			u8 chunk;
			u64 bitmask;
		};
		static voxel_ndces voxel_index_solver(vmath::vec<3, i32> uvw_floored) // Returns metachunk + chunk + bitmask to select specific voxels within chunks
			// [uvw_floored] is expected in voxel space (0...vol::width on each axis)
		{
			// Return payload
			voxel_ndces ret;

			// Compute voxel bitmask
			vmath::vec<3, i32> voxel_uvw = vmath::vec<3, i32>(uvw_floored.x() % metachunk::chunk_res_x,
				uvw_floored.y() % metachunk::chunk_res_y,
				uvw_floored.z() % metachunk::chunk_res_z);
			voxel_uvw *= vmath::vec<3, i32>(1, metachunk::chunk_res_y, metachunk::chunk_res_xy);
			ret.bitmask = ((1ull << voxel_uvw.z()) << voxel_uvw.y()) << voxel_uvw.x();

			// Compute chunk/metachunk indices
			ret.chunk = chunk_index_solver(uvw_floored);

			// Return
			return ret;
		}

		// Generic 3D index solver, assuming euclidean grid space and taking an index, width metric, and area metric
		template<u32 w, u32 a>
		static vmath::vec<3> expand_ndx(u32 ndx)
		{
			return vmath::vec<3>(static_cast<float>(ndx % w),
				static_cast<float>((ndx % a) / w),
				static_cast<float>(ndx / a));
		}

		// Generic 3D space flattener, implemented as the inverse of [expand_ndx(...)]
		template<u32 w, u32 a>
		static u32 flatten_ndx(vmath::vec<3> uvw)
		{
			float res = uvw.x() +
				(uvw.y() * w) +
				(uvw.z() * a);
			return static_cast<u32>(res);
		}

		// Resolve screen-space volume bounds for the current camera transform
		// (inverse lens sampling performed on the camera, so this just needs to resolve worldspace bounds and reproject them that way, before
		// taking the min/max coordinates in the 2D plane and storing those)
		static void resolveSSBounds(vmath::vec<2>(*inverse_lens_sampler_fn)(vmath::vec<3>))
		{
			// Resolve worldspace extents for our volume AABB
			// Will eventually need to adjust this code for different camera angles, zoom, panning, etc.
			const vmath::vec<3> vol_p = metadata->transf.pos;
			const vmath::vec<3> vol_extents = metadata->transf.scale * 0.5f;

			// Volume AABB vertices, from left->right, top->bottom, and front->back
			///////////////////////////////////////////////////////////////////////

			// Front
			const vmath::vec<3> aabb0 = vol_p + vmath::vec<3>(-vol_extents.x(), vol_extents.y(), -vol_extents.z());
			const vmath::vec<3> aabb1 = vol_p + vmath::vec<3>(vol_extents.x(), vol_extents.y(), -vol_extents.z());
			const vmath::vec<3> aabb2 = vol_p + vmath::vec<3>(-vol_extents.x(), -vol_extents.y(), -vol_extents.z());
			const vmath::vec<3> aabb3 = vol_p + vmath::vec<3>(vol_extents.x(), -vol_extents.y(), -vol_extents.z());

			// Back
			const vmath::vec<3> aabb4 = vol_p + vmath::vec<3>(-vol_extents.x(), vol_extents.y(), vol_extents.z());
			const vmath::vec<3> aabb5 = vol_p + vmath::vec<3>(vol_extents.x(), vol_extents.y(), vol_extents.z());
			const vmath::vec<3> aabb6 = vol_p + vmath::vec<3>(-vol_extents.x(), -vol_extents.y(), vol_extents.z());
			const vmath::vec<3> aabb7 = vol_p + vmath::vec<3>(vol_extents.x(), -vol_extents.y(), vol_extents.z());

			// Project AABB vertices into screenspace
			const vmath::vec<2> aabb_vertices_ss[8] = { inverse_lens_sampler_fn(aabb0),
														inverse_lens_sampler_fn(aabb1),
														inverse_lens_sampler_fn(aabb2),
														inverse_lens_sampler_fn(aabb3),
														inverse_lens_sampler_fn(aabb4),
														inverse_lens_sampler_fn(aabb5),
														inverse_lens_sampler_fn(aabb6),
														inverse_lens_sampler_fn(aabb7) };

			// Resolve screen-space quad from min/max vertices
			vmath::vec<4> min_max_px = vmath::vec<4>(9999.9f, 9999.9f, -9999.9f, -9999.9f); // Order is minX, minY, maxX, maxY
			for (u8 i = 0; i < 8; i++)
			{
				float vx = aabb_vertices_ss[i].e[0], vy = aabb_vertices_ss[i].e[1];
				min_max_px.e[0] = vmath::min(min_max_px.e[0], vx);
				min_max_px.e[1] = vmath::min(min_max_px.e[1], vy);
				min_max_px.e[2] = vmath::max(min_max_px.e[2], vx);
				min_max_px.e[3] = vmath::max(min_max_px.e[3], vy);
			}
			metadata->transf.ss_v0 = min_max_px.xy(); // Min, min
			metadata->transf.ss_v1 = vmath::vec<2>(min_max_px.x(), min_max_px.w()); // Min, max
			metadata->transf.ss_v2 = vmath::vec<2>(min_max_px.z(), min_max_px.y()); // Max, min
			metadata->transf.ss_v3 = min_max_px.zw(); // Max, max
		}
	};

	// Volume initializer, either loads voxels from disk or generates them procedurally on startup
	// Loop through metachunks, and for each metachunk run the existing bounds test;
	// if the bounds test fails/passes at the metachunk level set that metachunk directly and skip ahead, if it fails at the chunk level
	// set that chunk directly and skip ahead, etc.
	void geom_setup(u16 num_tiles_x, u16 num_tiles_y, u16 tile_ndx)
	{
		const u32 num_tiles = static_cast<u32>(num_tiles_x) * num_tiles_y;
		const u32 slice_width = vol::num_metachunks_z / num_tiles; // Width for most slices except the last one, good enough for offset maths
		const u32 init_z = static_cast<u32>(tile_ndx) * slice_width;
		const u32 max_z = vmath::min(init_z + slice_width, vol::num_metachunks_z - init_z);

		// Scale sphere into metachunk space
		constexpr float metachunk_ori = (vol::width / vol::metachunk::num_vox_x) / 2;
		constexpr float r = metachunk_ori;
		constexpr float r2 = r * r;
		const vmath::vec<3> circOrigin(metachunk_ori,
			metachunk_ori,
			metachunk_ori);

		// Scale z-slices into metachunk space
		u32 metachunk_ctr = init_z * vol::num_metachunks_xy;
		u32 metachunk_ndx_min = vol::flatten_ndx<vol::num_metachunks_x, vol::num_metachunks_xy>(vmath::vec<3>(0.0f, 0.0f, static_cast<float>(init_z)));
		u32 metachunk_ndx_max = vol::flatten_ndx<vol::num_metachunks_x, vol::num_metachunks_xy>(vmath::vec<3>(vol::num_metachunks_x - 1,
			vol::num_metachunks_y - 1,
			static_cast<float>(max_z)));

		// Set-up slices
#define TEST_NOISE_CUBE
//#define TEST_SOLID_CUBE
//#define TEST_SOLID_SPHERE
#if !defined(TEST_SOLID_CUBE) && !defined(TEST_SOLID_SPHERE)
		float sample[8];
#endif
		for (u32 i = metachunk_ndx_min; i < metachunk_ndx_max; i++)
		{
#ifndef TEST_SOLID_SPHERE
#ifndef TEST_NOISE_CUBE
#ifndef TEST_SOLID_CUBE
			const vmath::vec<3> uvw = vol::expand_ndx<vol::num_metachunks_x, vol::num_metachunks_xy>(i);
			const float d = (uvw - circOrigin).sqr_magnitude() - r2; // Sphere SDF
			sample[0] = 0.6f;
			if (d < 0.0f && sample[0] > 0.5f)
			{
				parallel::rand_streams[tile_ndx].next(sample);
				parallel::rand_streams[tile_ndx].next(sample + 4);
				u64* chunks = vol::metachunks[i].chunks;

				// Fuzzy version of the metachunk sphere above, with occasional fireflies
				float t = 1.0f - (std::abs(d) / r2);
				t *= t;
				const double soften = vmath::lerp<double>(static_cast<double>(0x33333333), static_cast<double>(0xffffffff), t);
				u64 soften_fac = static_cast<u64>(soften);
				soften_fac |= 0xffffffffull << 4ull;
				chunks[0] = static_cast<u64>(sample[0] * 0xffffffffffffffff) | soften_fac;
				chunks[1] = static_cast<u64>(sample[1] * 0xffffffffffffffff) | soften_fac;
				chunks[2] = static_cast<u64>(sample[2] * 0xffffffffffffffff) | soften_fac;
				chunks[3] = static_cast<u64>(sample[3] * 0xffffffffffffffff) | soften_fac;
				chunks[4] = static_cast<u64>(sample[4] * 0xffffffffffffffff) | soften_fac;
				chunks[5] = static_cast<u64>(sample[5] * 0xffffffffffffffff) | soften_fac;
				chunks[6] = static_cast<u64>(sample[6] * 0xffffffffffffffff) | soften_fac;
				chunks[7] = static_cast<u64>(sample[7] * 0xffffffffffffffff) | soften_fac;

				// Fuzzy plus sign
				//chunks[0] = static_cast<u64>(sample[0] * 0xffffffffffffffff) | 0x7777ffffffffffff;
				//chunks[1] = static_cast<u64>(sample[1] * 0xffffffffffffffff) | 0xffff7777ffffffff;
				//chunks[2] = static_cast<u64>(sample[2] * 0xffffffffffffffff) | 0xffffffff7777ffff;
				//chunks[3] = static_cast<u64>(sample[3] * 0xffffffffffffffff) | 0xffffffffffff7777;
				//chunks[4] = static_cast<u64>(sample[4] * 0xffffffffffffffff) | 0xffffffff7777ffff;
				//chunks[5] = static_cast<u64>(sample[5] * 0xffffffffffffffff) | 0xffff7777ffffffff;
				//chunks[6] = static_cast<u64>(sample[6] * 0xffffffffffffffff) | 0x7777ffffffffffff;
				//chunks[7] = static_cast<u64>(sample[7] * 0xffffffffffffffff) | 0x7f7f7f7f7f7f7f7f;

				// Kinda cool fuzzy oval shape - lke some kind of polarized sphere, noisy in the middle and bow shapes on either side
				//chunks[0] = static_cast<u64>(sample[0] * 0xffffffffffffffff) | 0x77777777ffffffff;
				//chunks[1] = static_cast<u64>(sample[1] * 0xffffffffffffffff) | 0xffffffff77777777;
				//chunks[2] = static_cast<u64>(sample[2] * 0xffffffffffffffff) | 0x77777777ffffffff;
				//chunks[3] = static_cast<u64>(sample[3] * 0xffffffffffffffff) | 0xffffffff77777777;
				//chunks[4] = static_cast<u64>(sample[4] * 0xffffffffffffffff) | 0x77777777ffffffff;
				//chunks[5] = static_cast<u64>(sample[5] * 0xffffffffffffffff) | 0xffffffff77777777;
				//chunks[6] = static_cast<u64>(sample[6] * 0xffffffffffffffff) | 0x77777777ffffffff;
				//chunks[7] = static_cast<u64>(sample[7] * 0xffffffffffffffff) | 0xffffffff77777777;
			}
			else
			{
				vol::metachunks[i].batch_assign(0);
			}
#else
			vol::metachunks[i].batch_assign(255);
#endif
#else
			// Load chunk data
			u64* chunks = vol::metachunks[i].chunks;

			// Structural noise for clumping effects without excessively fine detail
			// (which was sampled out in prior versions)

			// Generate random values
			parallel::rand_streams[tile_ndx].next(sample);
			parallel::rand_streams[tile_ndx].next(sample + 4);

			// Modulate first four chunks (first RNG tap)
			chunks[0] = static_cast<u64>(sample[0] * 0xffffffffffffffff) & 0x7fffffffffffffff;
			chunks[1] = static_cast<u64>(sample[1] * 0xffffffffffffffff) & 0x7fffffffffffffff;
			chunks[2] = static_cast<u64>(sample[2] * 0xffffffffffffffff) & 0x7fffffffffffffff;
			chunks[3] = static_cast<u64>(sample[3] * 0xffffffffffffffff) & 0x7fffffffffffffff;

			// Modulate next four chunks (second RNG tap)
			chunks[4] = static_cast<u64>(sample[4] * 0xffffffffffffffff) | 0x7fffffffffffffff;
			chunks[5] = static_cast<u64>(sample[5] * 0xffffffffffffffff) | 0x7fffffffffffffff;
			chunks[6] = static_cast<u64>(sample[6] * 0xffffffffffffffff) | 0x7fffffffffffffff;
			chunks[7] = static_cast<u64>(sample[7] * 0xffffffffffffffff) | 0x7fffffffffffffff;
#endif
#else
			const vmath::vec<3> uvw = vol::expand_ndx<vol::num_metachunks_x, vol::num_metachunks_xy>(i);
			const float d = (uvw - circOrigin).sqr_magnitude() - r2; // Sphere SDF
			u8 v = d < 0.0f ? 0xff : 0x0;
			//vol::metachunks[i].batch_assign(v);
			for (u32 j = 0; j < 8; j++)
			{
				vol::metachunks[i].chunks[j] = v;
			}
#endif
			// Populate metachunk occupancy data
			u8 occupancies = (vol::metachunks[i].chunks[0] > 0) |
				((vol::metachunks[i].chunks[1] > 0) << 1) |
				((vol::metachunks[i].chunks[2] > 0) << 2) |
				((vol::metachunks[i].chunks[3] > 0) << 3) |
				((vol::metachunks[i].chunks[4] > 0) << 4) |
				((vol::metachunks[i].chunks[5] > 0) << 5) |
				((vol::metachunks[i].chunks[6] > 0) << 6) |
				((vol::metachunks[i].chunks[7] > 0) << 7);
			vol::metachunk_occupancies[i] = occupancies;
		};
	}

	export void init(vmath::vec<2>(*inverse_lens_sampler_fn)(vmath::vec<3>))
	{
		// Allocate volume memory
		vol::metadata = mem::allocate_tracing<vol::vol_nfo>(sizeof(vol::vol_nfo)); // Generalized volume info
		vol::metachunks = mem::allocate_tracing<vol::metachunk>(vol::num_metachunks * sizeof(vol::metachunk)); // Generalized volume info
		vol::metachunk_occupancies = mem::allocate_tracing<u8>(vol::num_metachunks * sizeof(u8));

		// Load/generate geometry
//#define TIMED_GEOMETRY_UPLOAD
#ifdef TIMED_GEOMETRY_UPLOAD
		double geom_setup_t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
		parallel::launch(geom_setup);

		// Wait for loader threads to finish
		bool loaded = false;
		while (!loaded)
		{
			bool loadTest = true;
			for (u32 i = 0; i < parallel::numTiles; i++)
			{
				loadTest = loadTest && parallel::tiles[i].state->load() == platform::threads::SLEEPING;
			}
			loaded = loadTest;
			if (loaded) break;
		}
#ifdef TIMED_GEOMETRY_UPLOAD
		char log_message[128];
		std::snprintf(log_message, sizeof(log_message), "geometry loaded within %f seconds \n", std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() - geom_setup_t);
		OutputDebugStringA(log_message);
		DebugBreak();
#endif
		// Possible debugging helper for geometry here; build in a .png exporter, write out cells on a certain slice to black or white depending on activation status
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// Update transform metadata; eventually this should be loaded from disk
		// (position should probably be actually zeroed, there's no reason for users to modify it instead of moving the camera)
		vol::metadata->transf.pos = vmath::vec<3>(0.0f, 0.0f, 20.0f);
		vol::metadata->transf.orientation = vmath::vec<4>(0.0f, 0.0f, 0.0f, 1.0f);
		vol::metadata->transf.scale = vmath::vec<3>(4, 4, 4);
		vol::resolveSSBounds(inverse_lens_sampler_fn);

		// Update material metadata; eventually this should be loaded from disk
		materials::instance& boxMat = vol::metadata->mat;
		boxMat.material_type = material_labels::DIFFUSE;
		boxMat.roughness = 0.2f;
		boxMat.spectral_ior = vmath::fn<4, const float>(spectra::placeholder_spd);
		boxMat.spectral_response = vmath::fn<4, const float>(spectra::placeholder_spd);
	}

	export void spin(float xrot, float yrot)
	{
		vmath::vec<3> axes(std::sin(xrot), std::sin(yrot), 0.0f);
		vmath::vec<4> q_xrot(axes.x(), 0.0f, 0.0f, std::cos(xrot));
		vmath::vec<4> q_yrot(0.0f, axes.y(), 0.0f, std::cos(yrot));
		vol::metadata->transf.orientation = q_xrot.qtn_rotation_concat(q_yrot).qtn_rotation_concat(vol::metadata->transf.orientation);
	}

	export void zoom(float z)
	{
		z = vmath::max(z + 1.0f, 0.0f); // Keep z above 0.0f
		// (+ above 1.0f by default)
		vol::metadata->transf.scale *= z;
	}

	// Test the bounding geometry for the volume grid
	// Used to quickly mask out rays that immediately hit the sky or an external light source
	export bool test(vmath::vec<3> dir, vmath::vec<3>* ro_inout, geometry::vol::vol_nfo* vol_nfo_out)
	{
		// Compute transformed ray position & direction
		vmath::vec<3> pos = vol::metadata->transf.pos;
		vmath::vec<3> ro = *ro_inout - pos;
		vmath::vec<3> rd = dir.qtn_rotation_apply(vol::metadata->transf.orientation);

		// Intersection vmath adapted from
		// https://www.shadertoy.com/view/ltKyzm, itself adapted from the Scratchapixel tutorial here:
		// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

		// Synthesize box boundaries
		const vmath::vec<3> extents = vol::metadata->transf.scale * 0.5f; // Extents from object origin
		const vmath::vec<3> boundsMin = (pos - extents);
		const vmath::vec<3> boundsMax = (pos + extents);

		// Evaluate per-axis distances to each plane in the box
		// Not super sure why these divisions happen?
		// Should probably reread vmath for all this in general
		vmath::vec<3> plane_dists[2] =
		{
			boundsMin / dir,
			boundsMax / dir
		};

		// Keep near distances in [0], far distances in [1]
		const vmath::vec<3> dist0 = plane_dists[0];
		const vmath::vec<3> dist1 = plane_dists[1];
		plane_dists[0].e[0] = vmath::min(dist0.x(), dist1.x());
		plane_dists[0].e[1] = vmath::min(dist0.y(), dist1.y());
		plane_dists[0].e[2] = vmath::min(dist0.z(), dist1.z());
		plane_dists[1].e[0] = vmath::max(dist0.x(), dist1.x());
		plane_dists[1].e[1] = vmath::max(dist0.y(), dist1.y());
		plane_dists[1].e[2] = vmath::max(dist0.z(), dist1.z());

		// Evaluate scalar min/max distances for the given ray
		vmath::vec<2> sT = vmath::vec<2>(vmath::max(vmath::max(plane_dists[0].x(), plane_dists[0].y()), plane_dists[0].z()),
			vmath::min(vmath::min(plane_dists[1].x(), plane_dists[1].y()), plane_dists[1].z()));
		sT = vmath::vec<2>(vmath::min(sT.x(), sT.y()), vmath::max(sT.x(), sT.y())); // Keep near distance in [x], far distance in [y]

		// Resolve intersection status
		const bool isect = (plane_dists[0].x() < plane_dists[1].y() && plane_dists[0].y() < plane_dists[1].x() &&
			plane_dists[0].z() < sT.y() && sT.x() < plane_dists[1].z()) && (sT.e[0] > 0); // Extend intersection test to ignore intersections behind the current ray  (where the direction
		// to the intersection point is the reverse of the current ray direction)

// Write out intersection position + shared object info for successful intersections, then early-out
// (need to sort candidates by depth here)
		if (isect)
		{
			*ro_inout = *ro_inout + (dir * sT.e[0]);
			*vol_nfo_out = *vol::metadata;
			return true;
		}
		return false;
	}

	// Test for intersections with individual cells within the grid, using DDA
	// (single-cell steps within the volume grid, along a given direction)
	// Implemented following the guide in this tutorial
	// https://www.y()outube.com/watch?v=W5P8GlaEOSI
	// + this paper/blog
	// https://castingrays.blogspot.com/2014/01/voxel-rendering-using-discrete-ray.html
	// many thanks to the creators of both <3
	export bool cell_step(vmath::vec<3> dir, vmath::vec<3>* ro_inout, vmath::vec<3> uvw_in, vmath::vec<3, i32>* uvw_i_inout, vmath::vec<3>* n_out, bool primary_ray)
	{
		// Safety test!
		// Make sure any rays that enter this function have safe starting values
		// Also validate against error cases (incoming coordinates more than one unit from a boundary)
		vmath::vec<3, i32> uvw_floored = *uvw_i_inout;
		const vmath::vec<3> bounds_dist_max = uvw_in - vmath::vec<3>(vol::width, vol::width, vol::width);
		const vmath::vec<3, i32> bounds_dist_min = vmath::vec<3, i32>(0, 0, 0) - uvw_floored;
		if (vmath::allGreater(bounds_dist_max, 0.0f) || vmath::allGreater(bounds_dist_min, -1)) // coordinates equal to vol::width are still out of bounds :p
		{
			// Something funky, bad coordinates or generating coordinates when we're outside the scene bounding box
//#define VALIDATE_CELL_RANGES
#ifdef VALIDATE_CELL_RANGES
			if (vmath::anyGreater(bounds_dist_max, 1.0f) || vmath::anyGreater(bounds_dist_min, -1.0f))
			{
				DebugBreak();
			}
#endif

			// Most cases will be regular rounding error :D
			vmath::clamp(uvw_in, vmath::vec<3>(0.0f), vmath::vec<3>(vol::width - 1));
			vmath::clamp(uvw_floored, vmath::vec<3, i32>(0), vmath::vec<3, i32>(vol::width - 1));
		}

		// We only traverse primary rays with an empty starting cell (since otherwise we can return immediately)
		// For non-primary rays (bounce rays) we ignore the starting cell and iterate through any others along the ray
		// direction
		const u32 init_metachunk_ndx = vol::metachunk_index_solver(uvw_floored);
		const vol::voxel_ndces init_ndces = vol::voxel_index_solver(uvw_floored);
		u64 chunk_state = vol::metachunks[init_metachunk_ndx].chunks[init_ndces.chunk] & init_ndces.bitmask;
		bool traversing = primary_ray ? (chunk_state == 0) : true; // We want to traverse empty cells on hit, and bounce off full ones

		// Core traversal loop/immediate return
		if (traversing)
		{
			// Compute change in uvw per-axis
			const vmath::vec<3, float> d_uvw = vmath::vsgn(dir);

			// Compute x/y/z-derivatives
			const vmath::vec<3> safe_dir_axes = vmath::vmax(vmath::vabs(dir), vmath::vec<3>(0.01f)) * d_uvw;
			const vmath::vec<3> g = vmath::vec<3>((dir / safe_dir_axes.e[0]).magnitude(), // Scalarized directional derivative relative to x
				(dir / safe_dir_axes.e[1]).magnitude(), // Scalarized directional derivative relative to y
				(dir / safe_dir_axes.e[2]).magnitude()); // Scalarized directional derivative realtive to z

			// Distances to the first cell interval/boundary
			vmath::vec<3> t = vmath::vec<3>((dir.e[0] >= 0 ? (uvw_floored.e[0] + 1.0f - uvw_in.e[0]) : uvw_in.e[0] - uvw_floored.e[0]) * g.e[0],
				(dir.e[1] >= 0 ? (uvw_floored.e[1] + 1.0f - uvw_in.e[1]) : uvw_in.e[1] - uvw_floored.e[1]) * g.e[1],
				(dir.e[2] >= 0 ? (uvw_floored.e[2] + 1.0f - uvw_in.e[2]) : uvw_in.e[2] - uvw_floored.e[2]) * g.e[2]);

			// DDA
			//////

			u8 min_axis = 0; // Smallest axis in our traversal vector, used to determine which direction to step through in each tap
			u32 metachunk_ndx = init_metachunk_ndx; // Saved on metachunk intersection to simplify chunk lookups
			u32 chunk_ndx = init_ndces.chunk; // Saved on chunk intersection to simplify voxel lookups
			u8 current_metachunk = 1;
			u8 current_chunk_mask = 1;
			bool cell_found = false;
			enum TRAVERSAL_MODE
			{
				METACHUNK,
				CHUNK,
				VOXEL
			};
			TRAVERSAL_MODE mode = METACHUNK; // Is our DDA currently running on chunks, metachunks, or voxels?
			while (!cell_found)
			{
				// Minimize divergence from the ideal path through [dir] by always incrementing our smallest axis
				min_axis = t.x() < t.y() && t.x() < t.z() ? 0 :
					t.y() < t.x() && t.y() < t.z() ? 1 :
					/* d_pos.x() < d_pos.y() || d_pos.x() <= d_pos.z() */ 2 /* : 0*/;

				// Scale our step sizes differently for different traversal granularities
				const u32 dda_res = mode == METACHUNK ? vol::metachunk::num_vox_x :
					mode == CHUNK ? vol::metachunk::chunk_res_x :
					/*VOXEL ? */1;

				// We want to weight each continuous step by its axis' contribution to the slope of the ray direction
				t.e[min_axis] += g.e[min_axis] * dda_res; // Optional scale here for metachunk traversal

				// Update our current voxel coordinate
				uvw_floored.e[min_axis] += static_cast<i32>(d_uvw.e[min_axis] * dda_res); // Optional scale here for metachunk traversal

				// Ray escaped the volume :o
				if (vmath::anyGreater(uvw_floored, vol::width - 1) || vmath::anyLesser(uvw_floored, 0))
				{
					uvw_floored = vmath::clamp(uvw_floored, vmath::vec<3, i32>(0, 0, 0), vmath::vec<3, i32>(vol::width - 1, vol::width - 1, vol::width - 1));
					cell_found = false;
					break;
				}

				// We calculate metachunk indices in every branch, so might as well move that here
				u32 local_metachunk_ndx = vol::metachunk_index_solver(uvw_floored);

				// Successful intersections :D
				// (or possibly rays passing through lower levels without hitting anything - move back up to higher levels in that case)
				if (mode == METACHUNK)
				{
					metachunk_ndx = local_metachunk_ndx;
					u8 metachunk_data = vol::metachunk_occupancies[metachunk_ndx];
					if (metachunk_data)
					{
						current_metachunk = metachunk_data;
						mode = CHUNK;
					}
				}
				else if (mode == CHUNK)
				{
					chunk_ndx = vol::chunk_index_solver(uvw_floored);
					current_chunk_mask = 1 << chunk_ndx;

					if (metachunk_ndx != local_metachunk_ndx)
					{
						mode = METACHUNK;
						metachunk_ndx = local_metachunk_ndx;
					}
					else if ((current_metachunk & current_chunk_mask) > 0)
					{
						mode = VOXEL;
					}
					/* else { /* Regular chunk-level traversal *//* } */
				}
				else if (mode == VOXEL)
				{
					const vol::voxel_ndces ndces = vol::voxel_index_solver(uvw_floored);
					if (chunk_ndx != ndces.chunk)
					{
						if (metachunk_ndx != local_metachunk_ndx)
						{
							mode = METACHUNK;
						}
						else
						{
							mode = CHUNK;
						}
						chunk_ndx = ndces.chunk;
						metachunk_ndx = local_metachunk_ndx;
					}
					else
					{
						u64 current_chunk = vol::metachunks[local_metachunk_ndx].chunks[ndces.chunk];
						if ((current_chunk & ndces.bitmask) > 0ull)
						{
							cell_found = true;
							break;
						}
					}
				}
			}

			// Outputs :D
			// Only need to write these if we've traversed the grid, since they'll be the same as our inputs
			// otherwise
			////////////////////////////////////////////////////////////////////////////////////////////////

			// Derive normal from most recent step (thanks nightchild from GP!)
			*n_out = min_axis == 0 ? vmath::vec<3>(-d_uvw.e[0], 0.0f, 0.0f) :
				min_axis == 1 ? vmath::vec<3>(0.0f, -d_uvw.e[1], 0.0f) :
				/*min_axis == 2 ? */vmath::vec<3>(0.0f, 0.0f, -d_uvw.e[2])/* : vmath::vec<3>(0, 0, -1.0f)*/;

			// Output integer UVW coordinate
			// We've already clamped it if we needed to, so a direct copy here is fine
			*uvw_i_inout = uvw_floored;

			// Calculate worldspace position delta, then update output coordinate
			/////////////////////////////////////////////////////////////////////

			// Reverse of the math we did in [scene.ixx] to resolve intersected voxel coordinates
			/////////////////////////////////////////////////////////////////////////////////////

			// Original math from [scene.ixx]
			//rel_p = (curr_ray.ori - volume_nfo.transf.pos) + (volume_nfo.transf.scale * 0.5f); // Relative position from lower object corner
			//uvw = rel_p / volume_nfo.transf.scale; // Normalized UVW
			//uvw_scaled = uvw * geometry::vol::width; // Voxel coordinates! :D

			// Reversed math
			vmath::vec<3> ro = vmath::vec<3>(uvw_floored.x(), uvw_floored.y(), uvw_floored.z());// + (dir * t.magnitude()); // Apply position delta
			ro /= vol::width; // Back to UVW space (0...1)
			ro *= vol::metadata->transf.scale; // Back to object space
			ro -= vol::metadata->transf.scale * 0.5f; // Position relative to centre, not lower corner
			ro += vol::metadata->transf.pos; // Back to worldspace :D
			ro = vmath::clamp(ro,
				vmath::vec<3>(vol::metadata->transf.pos - vol::metadata->transf.scale * 0.5f),
				vmath::vec<3>(vol::metadata->transf.pos + vol::metadata->transf.scale * 0.5f));
			*ro_inout = ro;

			// Return cell discovery state
			// Failure isn't a scary error case anymore - just means that we bounced out of the volume :)
			return cell_found;
		}
		else
		{
			// If we're testing a primary ray, and the current UVW coordinate is filled, we've instantly found a cell :p
			return true;
		}
	}
};

#ifdef GEOMETRY_DBG
#pragma optimize("", on)
#endif

u8* geometry::vol::metachunk_occupancies;
geometry::vol::metachunk* geometry::vol::metachunks;
geometry::vol::vol_nfo* geometry::vol::metadata;
