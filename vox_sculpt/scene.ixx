module;
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#undef min
#undef max

export module scene;

import vmath;
import path;
import vox_ints;
import geometry;
import parallel;
import materials;
import spectra;
import lights;

// Project is now too complex for regular debug mode
// oof
//#define SCN_DBG
#ifdef SCN_DBG
#pragma optimize("", off)
#endif
export namespace scene
{
    // Traverse the scene, pass path vertices back up to our pipeline so they can be integrated separately from scene traversal
    // (allowing for BDPT/VCM and other integration schemes besides regular unidirectional)
    void isect(tracing::path_vt init_vt, tracing::path* vertex_output, float* isosurf_dist, u32 tileNdx)
    {
        float horizon_dist = 1000.0f;
        typedef tracing::path_vt ray;
        ray curr_ray(init_vt);
        ray out_vt = curr_ray; // Output vertex for each bounce is always one iteration behind the current ray (since those are intended to be setting up the _next_ bounce in each path)
        bool path_absorbed = false;
        bool path_escaped = false;
        u8 bounceCtr = 0;
        geometry::vol::vol_nfo volume_nfo;
        bool within_grid = geometry::test(curr_ray.dir, &curr_ray.ori, &volume_nfo);
        //#define VALIDATE_VERTEX_COUNTS
        //#define PROPAGATION_DBG
        //#define VALIDATE_BOUNCES
//#define VALIDATE_BOUNDARY_SCATTERING
#ifdef VALIDATE_BOUNCES
        bool validating_path_bounces = false;
#endif
        //bool mask_current_cell = false; // Flag preventing self-intersection when we're bouncing out of volume cells
#ifdef VALIDATE_VERTEX_COUNTS
        u32 init_vt_count = vertex_output->front;
#endif
        while (!path_absorbed && !path_escaped)
        {
            // Test for intersection
            bool first_grid_hit = (bounceCtr == 0 && within_grid); // If we've just hit the volume grid, set this so we can use stored distances to avoid marching towards isoboundaries
            if (within_grid) // Within the grid, march until we leave the bounding box
            {
                // March/jump to intersections within the voxel grid
                /////////////////////////////////////////////////////////

                // Special case for silhouettes; if we've just hit the grid boundaries, jump to the surface instead of recalculating that distance every sample
                if (first_grid_hit && *isosurf_dist > -1.0f) // Isosurface distances initialize to [-1]; zero distances are reserved for subpixels that immediately touch a grid boundary
                {
                    curr_ray.ori += curr_ray.dir * *isosurf_dist;
                }

                // Sometimes rays land "outside" the volume because of floating-point error; normalize those cases here
                curr_ray.ori = vmath::clamp(curr_ray.ori,
                                            vmath::vec<3>(volume_nfo.transf.pos - volume_nfo.transf.scale * 0.5f),
                                            vmath::vec<3>(volume_nfo.transf.pos + volume_nfo.transf.scale * 0.5f));

                // March from the current position to the first cell with non-zero state
                ////////////////////////////////////////////////////////////////////////

                // Initialize useful variables
                bool surf_found = false; // Traversal control switch; testing the whole grid up here means that not finding a surface is equivalent to having a ray escape
                const vmath::vec<3> grid_isect_pos = curr_ray.ori; // Cached intersection position to allow resolving exact primary ray distance later on
                vmath::vec<3> voxel_normal = vmath::vec<3>(0, 0, -1); // Normals, initialized to something mostly safe (for the default z-aligned view anyways)

                // Resolve intersection cell each tap
                vmath::vec<3> rel_p = (curr_ray.ori - volume_nfo.transf.pos) + (volume_nfo.transf.scale * 0.5f); // Relative position from lower object corner
                vmath::vec<3> uvw = rel_p / volume_nfo.transf.scale; // Normalized UVW
                vmath::vec<3> uvw_scaled = uvw * geometry::vol::width; // Voxel coordinates! :D
                vmath::vec<3, i32> uvw_i = vmath::vec3_cast<vmath::vec<3>, vmath::vec<3, i32>>(vmath::vfloor(vmath::vabs(uvw_scaled))); // Probably paranoid, but voxel coordinates should never be negative

                // Find the next cell intersection, and step into it before recalculating rel_p/uvw & checking occupancy again
                // Using DDA means that cell steps travel directly to geometry, so any stepping failures mean that our rays leave geometry completely
//#define VALIDATE_STEPPED_RO
#ifdef VALIDATE_STEPPED_RO
                vmath::vec<3> ro_input = curr_ray.ori;
#endif
                const bool cell_step_success = geometry::cell_step(curr_ray.dir, &curr_ray.ori, uvw_scaled, &uvw_i, &voxel_normal, first_grid_hit);
                if (!cell_step_success) // No intersections along the given direction :(
                {
                    uvw_i = vmath::vmax(uvw_i, vmath::vec<3, i32>(0, 0, 0));
                    within_grid = false;
                }
                /*else
                {
                    // Intersection found :D
                    // We can move on to sampling the current bounce further down
                }*/

#ifdef VALIDATE_STEPPED_RO
                if (vmath::anyGreater(curr_ray.ori, geometry::vol::width - 1) || vmath::anyLesser(curr_ray.ori, -2.0f))
                {
                    DebugBreak();
                }
#endif

                // Update isosurface distances on first grid intersections
                if (first_grid_hit)
                {
                    *isosurf_dist = (curr_ray.ori - grid_isect_pos).magnitude();
                }

                // Skip remaining tracing work if we've left the grid boundaries
                if (within_grid)
                {
                    // Sample surfaces (just lambertian diffuse for now)
                    switch (volume_nfo.mat.material_type)
                    {
                        case material_labels::DIFFUSE:
                            float sample[4];
                            parallel::rand_streams[tileNdx].next(sample);
                            materials::diffuse_surface_sample(&out_vt.dir, &out_vt.pdf, sample[0], sample[1]);
                            materials::diffuse_lambert_reflection(out_vt.rho_sample, volume_nfo.mat.spectral_response, curr_ray.ori, &out_vt.power, &out_vt.rho_weight);
                            break;
                        default:
                            DebugBreak(); // Unsupported material ;_;
                            break;
                    }

                    // Process absorption/extinction, either spectral (zero rho_weight), volumetric/environmental (zero power), or statistical
                    // (zero probability)
                    // Technically this should be unnecessary with a solid diffuse volume...but my voxels are so small that I'm getting precision issues
                    // on which normals to use and seeing rays intermittently get lost inside the surface. It feels better to give in reality there and
                    // treat my voxels like diffuse flakes instead of trying to force them into clean boxes that never experience translucency
                    // (it's definitely not physically accurate translucency but eh, like, self-intersections seem quite rare, I'm not sure how problematic
                    // that will be in practice)
                    if (curr_ray.rho_weight <= vmath::eps ||
                        curr_ray.power <= vmath::eps ||
                        curr_ray.pdf <= vmath::eps)
                    {
                        path_absorbed = true;
                    }
                    else
                    {
                        // Map outgoing direction back from sampling-space to worldspace
                        vmath::m3 nSpace = vmath::normalSpace(voxel_normal);
                        out_vt.dir = nSpace.apply(out_vt.dir).normalized();
                        out_vt.ori = curr_ray.ori;
                        out_vt.mat = &volume_nfo.mat;

                        // Cache path vertex for integration
                        vertex_output->push(out_vt);
#ifdef VALIDATE_BOUNCES
#ifdef VALIDATE_BOUNCE_WORLD_COORDS
                        parallel::append_tile_log(tileNdx, "bounce occurred at %f, %f, %f, \n", out_vt.ori.x(), out_vt.ori.y(), out_vt.ori.z())
#else
                        parallel::append_tile_log(tileNdx, "bounce occurred at %f, %f, %f, \n", uvw_i.x(), uvw_i.y(), uvw_i.z());
                        validating_path_bounces = true;
#endif
#endif
#ifdef VALIDATE_VERTEX_COUNTS
                        parallel::append_tile_log(tileNdx, "vertex count %i, initially %i, \n", vertex_output->front, init_vt_count);
#endif
                        // Update current ray
                        curr_ray = out_vt;
                    }
                }
            }
            else
            {
                // Map exiting rays onto the sky (set 1000 units from the scene origin)
                out_vt.ori = curr_ray.ori + (curr_ray.dir * lights::sky_dist);

                // Sample our sky model (super wip)
//#define DEBUG_RED_SKY
#ifdef DEBUG_RED_SKY
                out_vt.rho_sample = 0.8f;
                out_vt.rho_weight = 1.0f;
#else
                out_vt.rho_weight = spectra::sky(out_vt.rho_sample, out_vt.dir.e[1]);
                out_vt.power *= lights::sky_env(&out_vt.pdf);
#endif

                // Pass exiting rays into our path buffer
                vertex_output->push(out_vt);
                path_escaped = true;

#ifdef VALIDATE_BOUNCES
                if (validating_path_bounces)
                {
                    parallel::append_tile_log(tileNdx, "ray escaped\n");
                }
#endif
            }

            // Update bounce counter (escaped rays still bounce exactly once off the sky)
            bounceCtr++;
        }
    }
};
#ifdef SCN_DBG
#pragma optimize("", on)
#endif