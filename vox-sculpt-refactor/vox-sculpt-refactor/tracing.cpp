
#include "camera.h"
#include "path.h"
#include "ui.h"
#include "mem.h"
#include "vmath.h"
#include "parallel.h"
#include <stdint.h>
#include "platform.h"
#include "traversal.h"
#include "geometry.h"
#include "spectra.h"
#include "sampler.h"
#include "aa.h"
#include "tracing.h"

#include "../../animuspix-libs/SimpleTiling/SimpleTiling/SimpleTiling.h"

tracing::path* cameraPaths; // One unique path/sensel - easier to parallelise, friendlier to VCM (when I eventually get around to it)
//tracing::path* lightPaths;
float* isosurf_distances; // Distances to sculpture boundaries from grid bounds, per-subpixel, refreshed on camera zoom/rotate + animation timesteps (if/when I decide to implement those)
uint32_t* sample_ctr_per_sensel = nullptr;
vmath::vec<2>* tracing_tile_positions = nullptr;
vmath::vec<2>* tracing_tile_bounds = nullptr;
vmath::vec<2>* tracing_tile_sizes = nullptr;

// Stratified spectra per-pixel, allowing us to bias color samples towards the scene SPD instead of drawing randomly for every tap
spectra::spectral_buckets* spectral_strata = nullptr;

// Tile width/height/areas, useful for clearing data & deriving tile indices
uint32_t tile_width = 0;
uint32_t tile_height = 0;
uint32_t tile_area = 0;

// + reciprocal tile width (num-tiles-x) and tile height (num-tiles-y)
uint32_t tiles_x = 0;
uint32_t tiles_y = 0;

// Tile draw mask - useful for culling work submission on fully-sampled tiles (mostly in preview mode)
std::atomic<uint64_t> tile_mask = UINT64_MAX;

// Set the render mode to use in the next "frame"
tracing::integration::RENDER_MODES renderMode = tracing::integration::RENDER_MODE_EDIT; // Edit mode by default
void tracing::integration::set_render_mode(RENDER_MODES mode)
{
    renderMode = mode;
}

// Clear render state (needed for render mode transitions + refreshing before each EDIT pass)
void tracing::integration::clear_render_state()
{
    const uint32_t numTiles = parallel::GetNumTilesTotal();

    platform::osClearMem(cameraPaths, sizeof(path) * numTiles);
    platform::osClearMem(spectral_strata, sizeof(spectra::spectral_buckets) * ui::window_area);
    platform::osClearMem(&tile_mask, sizeof(uint64_t));
    platform::osClearMem(sample_ctr_per_sensel, sizeof(uint32_t) * ui::window_area);

    for (uint32_t i = 0; i < numTiles; i++)
    {
        for (int32_t j = 0; j < tile_area; j++)
        {
            isosurf_distances[(i * j) + j] = -1.0f;
        }
    }
}

// Core image integrator - shared across all render modes for simplicity
// The idea is that every mode gets here eventually, but they vary in how pipelined they are and whether they send
// updates to the window
void tracing::integration::image_integrator()
{
    // Broadcast draw work over all remaining tiles, using the mask we composed before
    simple_tiling::submit_draw_work([](simple_tiling_utils::v_type pixels, simple_tiling_utils::color_batch* output)
    {
        // Compute tile index from current pixels (all 8 should be in the same tile)
        const float window_width = ui::window_width;
        auto wvec = v_op(set1_ps)(window_width);
        auto yvec = v_op(floor_ps)(v_op(div_ps)(pixels, wvec));
        auto xvec = v_op(sub_ps)(pixels, v_op(mul_ps)(yvec, wvec));

        float _pixels[NUM_VECTOR_LANES] = {};
        float pixels_x[NUM_VECTOR_LANES] = {};
        float pixels_y[NUM_VECTOR_LANES] = {};

        v_op(store_ps)(_pixels, pixels);
        v_op(store_ps)(pixels_x, xvec);
        v_op(store_ps)(pixels_y, yvec);

        uint32_t tile_y = pixels_y[0] / tile_height;
        uint32_t tile_x = pixels_x[0] / tile_width;
        uint32_t tile_ndx = (tile_y * tiles_x) + tile_x;

        // Eventually consistent oversampling test, to account for interlacing;
        // if all pixels/batches are fully sampled, none of them should take
        // the else branch and the tile will eventually be dropped from the
        // dispatch mask
        uint32_t batch_samples = sample_ctr_per_sensel[uint32_t(_pixels[0])];
        if (batch_samples > aa::max_samples)
        {
            tile_mask &= ~(1ull << tile_ndx);
            return;
        }
        else
        {
            tile_mask |= (1ull << tile_ndx);
        }

        #define DEMO_SPECTRAL_PT
//#define DEMO_FILM_RESPONSE
//#define DEMO_XOR
//#define DEMO_NOISE
//#define DEMO_AND
//#define DEMO_HYPERBOLA
#ifdef DEMO_HYPERBOLA
        float distX = ((float)abs(abs(x - ui::image_centre_x) - ui::image_centre_x)) / (float)ui::image_centre_x;
        float distY = ((float)abs(abs(y - ui::image_centre_y) - ui::image_centre_y)) / (float)ui::image_centre_y;
        float dist = 1.0f - sqrt(distX * distX + distY * distY);
        camera::sensor_response(dist, 1.0f, pixel_ndx, sample_ctr[tile_ndx]);
        camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_AND)
        float rho = (x & y) / (float)y;
        camera::sensor_response(rho, 1.0f, pixel_ndx, sample_ctr[tile_ndx]);
        camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_XOR)
        float rho = ((x % 1024) ^ (y % 1024)) / 1024.0f;
        camera::sensor_response(rho, 1.0f, pixel_ndx, sample_ctr[tile_ndx]);
        camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_NOISE)
        float sample[4];
        parallel::rand_streams[tile_ndx].next(sample); // Three wasted values :(
        camera::sensor_response(sample[0], 1.0f, pixel_ndx, sample_ctr[tile_ndx]);
        camera::tonemap_out(pixel_ndx);
#elif defined (DEMO_FILM_RESPONSE) // Rainbow gradient test
        // Should try to vectorize these functions in future
        for (uint32_t i = 0; i < NUM_VECTOR_LANES; i++)
        {
            camera::sensor_response((float)pixels_x[i] / (float)ui::window_width, 1.0f / aa::max_samples, 1.0f, 1.0f, _pixels[i], batch_samples);
            output->colors8bpc[i] = camera::tonemap_out(_pixels[i]);
            sample_ctr_per_sensel[uint32_t(_pixels[i])]++;
        }
#elif defined (DEMO_SPECTRAL_PT)
        // Scalar loop for now - eventually we want to vectorize this
        for (uint32_t i = 0; i < NUM_VECTOR_LANES; i++)
        {
            // Draw random values for lens sampling
            float sample[4];
            parallel::GetRNGStream(tile_ndx).next(sample);

            const uint32_t pixel_ndx = (uint32_t)_pixels[i];
            const uint32_t path_ndx = (tile_ndx * NUM_VECTOR_LANES) + i;

            // Resolve a spectral sample for the current pixel
            float s = spectral_strata[pixel_ndx].draw_sample(sample[0], sample[1]);

            // Intersect the scene/the background
            float rho, pdf, rho_weight, power;
            const path_vt cam_vt = camera::lens_sample((float)pixels_x[i], (float)pixels_y[i], sample[2], sample[3], s);
            traversal::iterate(cam_vt, cameraPaths + path_ndx, isosurf_distances + pixel_ndx, tile_ndx);
            //scene::isect(lights::sky_sample(x, y, sample[0]), lightPaths[tile_ndx]);

            // Integrate scene contributions (unidirectional for now)
            cameraPaths[path_ndx].resolve_path_weights(&rho, &pdf, &rho_weight, &power); // Light/camera path merging decisions are performed while we integrate camera paths,
            // so we only need to resolve weights for one batch

            // Remove the current path from the backlog for this tile
            // (BDPT/VCM implementation will delay this until after separately tracing every path)
            cameraPaths[path_ndx].clear();
            //lightPaths[tile_ndx].clear();

            // Test for unresolved/buggy paths
            //if (cameraPaths[tile_ndx].front > 0) platform::osDebugBreak();
            // Compute sensor response + apply sample weight (composite of integration weight for spectral accumulation,
            // lens-sampled filter weight for AA, and path index weights from ray propagation)
            camera::sensor_response(rho, rho_weight, pdf, power, pixel_ndx, sample_ctr_per_sensel[pixel_ndx]);

            // Map resolved sensor responses back into tonemapped RGB values we can store for output
            output->colors8bpc[i] = camera::tonemap_out(pixel_ndx);
            sample_ctr_per_sensel[pixel_ndx]++;

            // Update weight for the bucket containing the current spectral sample
            spectral_strata[pixel_ndx].update(rho_weight);
        }
#endif
    }, tile_mask);
}

void tracing::integration::init()
{
    // Allocate tracing arrays
    const uint32_t numTiles = parallel::GetNumTilesTotal();
    const uint32_t numParallelPaths = NUM_VECTOR_LANES * numTiles;
    cameraPaths = mem::allocate_tracing<path>(sizeof(path) * numParallelPaths);
    sample_ctr_per_sensel = mem::allocate_tracing<uint32_t>(sizeof(uint32_t) * ui::window_area);
    tracing_tile_positions = mem::allocate_tracing<vmath::vec<2>>(sizeof(vmath::vec<2>) * numTiles);
    tracing_tile_bounds = mem::allocate_tracing<vmath::vec<2>>(sizeof(vmath::vec<2>) * numTiles);
    tracing_tile_sizes = mem::allocate_tracing<vmath::vec<2>>(sizeof(vmath::vec<2>) * numTiles);
    isosurf_distances = (float*)mem::allocate_tracing<float>(sizeof(float) * ui::window_area); // Eventually this will be per-subpixel instead of per-macropixel

    // Allocate & initialize spectral strata
    spectral_strata = mem::allocate_tracing<spectra::spectral_buckets>(sizeof(spectra::spectral_buckets) * ui::window_area);
    for (uint32_t i = 0; i < ui::window_area; i++)
    {
        spectral_strata[i].init(); // Reserve zero distance for voxels directly facing a grid boundary
    }

    // Resolve tracing types
    platform::osClearMem(cameraPaths, sizeof(path) * numParallelPaths);
    for (uint32_t i = 0; i < ui::window_area; i++)
    {
        isosurf_distances[i] = -1.0f; // Reserve zero distance for voxels directly facing a grid boundary
    }

    // Zero sample counters
    platform::osClearMem(sample_ctr_per_sensel, sizeof(uint32_t) * ui::window_area);

    // Resolve tile dimensions
    tiles_x = parallel::GetNumTilesX();
    tiles_y = parallel::GetNumTilesY();

    tile_width = ui::window_width / tiles_x;
    tile_height = ui::window_height / tiles_y;
    tile_area = tile_width * tile_height;
}
