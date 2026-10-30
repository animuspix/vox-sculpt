export module camera;

import ui;
import mem;
import path;
import vmath;
import scene;
import spectra;
import vox_ints;
import aa;
import <cmath>;
import <cstring>;

// Digital color/sensor data model, used to map from spectral inputs to writable screen colors
// (slowly turning into a retinal cone cell model instead oops, might update names eventually)
struct sensel
{
    sensel(float _r, float _g, float _b) :
        r(_r), g(_g), b(_b) {}
    sensel operator+(const sensel& rhs)
    {
        return sensel(rhs.r + r, rhs.g + g, rhs.b + b);
    }
    sensel operator*(const float rhs) const
    {
        return sensel(r * rhs, g * rhs, b * rhs);
    }
    sensel operator/(const float rhs) const
    {
        return sensel(r / rhs, g / rhs, b / rhs);
    }
    float r;
    float g;
    float b;
};

export namespace camera
{
    // Intermediate sensor values for filtering/anti-aliasing/temporal integration
    sensel* sensor_grid;
    constexpr u32 sensor_grid_footprint = ui::window_width * ui::window_height * sizeof(sensel);

    // Screen color/picture data (8bpc)
    u32* digital_colors;
    constexpr u32 digital_colors_footprint = ui::window_width * ui::window_height * sizeof(u32);

    // Sum of the filter values used for each sample, per-sensor
    // Needed for correct image integration when I'm using non-boxy filters, since those filters
    // can integrate to more/less than 1 and (=> and lead to weird results unless you divide out
    // the sum of the filter values each frame)
    float* filter_sum_grid;
    constexpr u32 filter_sum_grid_footprint = ui::window_width * ui::window_height * sizeof(float);

    // Camera sampling! just perspective projection for now :)
    constexpr float FOV_RADS = vmath::pi * 0.5f;
    const vmath::vec<3> camera_pos() { return vmath::vec<3>(0, 0, -10.0f); }
    const float camera_z_axis() { return (ui::window_width * aa::samples_x) / std::tan(FOV_RADS * 0.5f); }
    export tracing::path_vt lens_sample(u32 film_x, u32 film_y, float rand_u, float rand_v, float rho)
    {
        // Compute supersampled coordinates
        vmath::vec<2> film_xy = aa::supersample(float(film_x), float(film_y));

        // Compute local sample coordinate
        const vmath::vec<2> subpx_xy = aa::jitter(rand_u, rand_v);

        // Compute filter weight, accumulate for the sampled sensor
        float filt = aa::blackman_harris_weight(subpx_xy);
        filter_sum_grid[film_x + film_y * ui::window_width] += filt;

        // Apply sample offset into film offset
        film_xy += subpx_xy;

        // Return weighted lens direction :D
        const vmath::vec<3> c = camera_pos();
        // Account for camera rotation here...
        return tracing::path_vt(vmath::vec<3>(film_xy.x() - ui::image_centre_x * aa::samples_x,
                                              film_xy.y() - ui::image_centre_y * aa::samples_y,
                                              camera_z_axis()).normalized(), // Probably don't need to normalize here, but the stability feels nice
                                              c, 1.0f, rho, 1.0f, 1.0f * filt);
    }

    // Find the perspective-projected pixel coordinate passing through the given worldspace 3D coordinate
    export vmath::vec<2> inverse_lens_sample(vmath::vec<3> world_pos)
    {
        // This algorithm traps world_pos within the projected camera vertices, and assumes a pinhole camera
        // This works because rays within the projected lens stay within the distribution of our lens for the
        // given FOV. Previous versions traced worldspace points back to their intersection with the camera's
        // initial projection plane (at camera_z_axis()), which proeduced distortion since there was no
        // guarantee the line between world points and camera pixels would be parallel with camera rays
        // intersecting the same points (especially for very wide or very narrow FOV)
        // Anither issue was that pixels never actually sit on the camera plane - they're created out there,
        // then immediately normalizeed, which would be fine if the rays projected back from our worldspace
        // points were coherent with the frustum, but as above, they could easily not be - so hidden points
        // could still read as visible after stretching them to the camera plane, and then map onto invalid
        // pixels when the gradient towards them is much lower/higher than the gradient of the nearest camera
        // ray (causing the stretch to displace their x/y coordinates less than it would have otherwise)
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        // Define each camera corner (normalized, giving a direction from the camera origin)
        i32 samples_x = i32(aa::samples_x);
        i32 samples_y = i32(aa::samples_y);
        vmath::vec<3> corners[4] = { vmath::vec<3>(-ui::image_centre_x * samples_x, ui::image_centre_y * samples_y, camera_z_axis()).normalized(), // Upper-left
                                     vmath::vec<3>(ui::image_centre_x * samples_x, ui::image_centre_y * samples_y, camera_z_axis()).normalized(), // Upper-right
                                     vmath::vec<3>(-ui::image_centre_x * samples_x, -ui::image_centre_y * samples_y, camera_z_axis()).normalized(), // Lower-left
                                     vmath::vec<3>(ui::image_centre_x * samples_x, -ui::image_centre_y * samples_y, camera_z_axis()).normalized() }; // Lower-right

        // Extend corners out to the z-plane containing world_pos
        for (u32 i = 0; i < 4; i++)
        {
            float dx = corners[i].x() / corners[i].z();
            float dy = corners[i].y() / corners[i].z();
            corners[i].e[0] = dx * world_pos.z();
            corners[i].e[1] = dy * world_pos.z();
            corners[i].e[2] = world_pos.z();
        }

        // Compute world_pos position relative to the lower-right corner of the projected lens
        vmath::vec<3> rel_p0 = world_pos - corners[2];

        // Normalize relative XY coordinates
        vmath::vec<2> wh = vmath::vabs(corners[0].xy() - corners[3].xy());
        vmath::vec<2> uv = rel_p0.xy() / wh;

        // Return scaled coordinates
        return vmath::vec<2>(uv.x() * ui::window_width,
                             uv.y() * ui::window_height);
    }

    // Combination of a custom sensor response curve (see spectra.h) and a basic integration scheme
    //
    // Integration scheme is very simple for now, just adding filtered colors together up to the max sample count;
    // future versions will use a more sophisticated filter with no decay (so late samples will be implicitly
    // weighted the same as early ones) and I might eventually consider performing integration in spectral space
    // instead (not 100% sure what that would look like)
    void sensor_response(float rho, float rho_weight, float pdf, float power, u32 ndx, u32 sample_num)
    {
        // Resolve responses per-channel
        vmath::vec<3> rgb = spectra::film(rho) * rho_weight * (power / pdf);

        // Compose isolated colours into a sensor value, apply accumulated weights
        // (from path-tracing + spectral integration), write to sensor output :)
        sensel& curr_sensel = sensor_grid[ndx]; // Read in current sensor value
        if (sample_num == 1)
        {
            curr_sensel.r = rgb.e[0];
            curr_sensel.g = rgb.e[1];
            curr_sensel.b = rgb.e[2];
        }
        else if (sample_num < aa::max_samples)
        {
            curr_sensel.r += rgb.e[0];
            curr_sensel.g += rgb.e[1];
            curr_sensel.b += rgb.e[2];
        }
    }

    // Tonemap + write to digital output :)
    export void tonemap_out(u32 ndx)
    {
        // Tonemap into integer 8bpc + return
        // Tonemapped with ACES, source:
        // https://www.shadertoy.com/view/WdjSW3
        // originally Narkowicz 2015, "ACES Filmic Tone Mapping Curve"
        auto aces = [](float x) {
            const float a = 2.51f;
            const float b = 0.03f;
            const float c = 2.43f;
            const float d = 0.59f;
            const float e = 0.14f;
            return (x * (a * x + b)) / (x * (c * x + d) + e);
        };
        sensel sensor_v = sensor_grid[ndx] / filter_sum_grid[ndx];
        const u32 r = static_cast<u32>(aces(vmath::clamp(sensor_v.r, 0.0f, 1.0f)) * 256);
        const u32 g = static_cast<u32>(aces(vmath::clamp(sensor_v.g, 0.0f, 1.0f)) * 256);
        const u32 b = static_cast<u32>(aces(vmath::clamp(sensor_v.b, 0.0f, 1.0f)) * 256);
        const u32 a = 0x0;
        digital_colors[ndx] = b |
                             (g << 8) |
                             (r << 16) |
                             (a << 24);
    }

    // Reconstruct sparse image data; useful if we want to undersample for faster renders
    // Current reconstruction uses bilinear interpolation, though other algorithms might be used in the future - the only
    // requirement is that algorithms are separable and progressive, since we want to perform our reconstruction on the image
    // data as its generated (to allow for immediate clean output, instead of pixellated output that suddenly cleans up after
    // the last sample)
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Switches to change between reconstruction algorithms at preprocessing time
//#define RECONSTRUCT_BILINEAR
//#define RECONSTRUCT_BILINEAR_SQUARE
#define RECONSTRUCT_NEAREST

    // Reconstruction on the x-axis operates on individual pixel-strips at a time, so we recover samples
    // on either side of the most recent stride before filling in the gaps between them
    export void reconstruct_x(u32 x, u32 ndx, u32 stride)
    {
        // Resolve left sample components
        const u32 l_sample = digital_colors[ndx - stride]; // No need to use the full map, since we know our blends will always be in the same pixel band
        const float l_red = ((l_sample & 0x00ff0000) >> 16) / 255.0f;
        const float l_green = ((l_sample & 0x0000ff00) >> 8) / 255.0f;
        const float l_blue = (l_sample & 0x000000ff) / 255.0f;
        const float l_alpha = ((l_sample & 0xff000000) >> 24) / 255.0f;

        // Resolve right sample components
        const u32 r_sample = digital_colors[ndx];
        const float r_red = ((r_sample & 0x00ff0000) >> 16) / 255.0f;
        const float r_green = ((r_sample & 0x0000ff00) >> 8) / 255.0f;
        const float r_blue = (r_sample & 0x000000ff) / 255.0f;
        const float r_alpha = ((r_sample & 0xff000000) >> 24) / 255.0f;

        // Interpolate
        i32 ndxori = ndx - stride; // Using the same trick as above, since we know our pixel band is constant
        const i32 xori = x - stride;
        i32 x_i = static_cast<i32>(x);
        for (i32 x2 = xori; x2 < x_i; x2++)
        {
            const float t = static_cast<float>((x2 - xori)) / stride;
#ifdef RECONSTRUCT_BILINEAR
            const u32 s_red = static_cast<u32>(vmath::lerp(l_red, r_red, t) * 255.5f);
            const u32 s_green = static_cast<u32>(vmath::lerp(l_green, r_green, t) * 255.5f);
            const u32 s_blue = static_cast<u32>(vmath::lerp(l_blue, r_blue, t) * 255.5f);
            const u32 s_alpha = static_cast<u32>(vmath::lerp(l_alpha, r_alpha, t) * 255.5f);
            const u32 s = s_blue |
                         (s_green << 8) |
                         (s_red << 16) |
                         (s_alpha << 24);
#else
#ifdef RECONSTRUCT_BILINEAR_SQUARE
            const u32 s_red = static_cast<u32>(vmath::lerp(l_red, r_red, t * t) * 255.5f);
            const u32 s_green = static_cast<u32>(vmath::lerp(l_green, r_green, t * t) * 255.5f);
            const u32 s_blue = static_cast<u32>(vmath::lerp(l_blue, r_blue, t * t) * 255.5f);
            const u32 s_alpha = static_cast<u32>(vmath::lerp(l_alpha, r_alpha, t * t) * 255.5f);
            const u32 s = s_blue |
                         (s_green << 8) |
                         (s_red << 16) |
                         (s_alpha << 24);
#else

#ifdef RECONSTRUCT_NEAREST
            const u32 s = (t <= 0.5f) ? l_sample : r_sample;
#endif
#endif
#endif
            camera::digital_colors[ndxori++] = s;
        }
    }

    // Y-component of regular two-pass reconstruction; blends together the rows generated
    // by reconstructing the x-component (see above)
    export void reconstruct_y(u32 y, u32 stride, u32 minX, u32 xMax)
    {
        for (u32 sample_x = minX; sample_x < xMax; sample_x++)
        {
            // Resolve upper sample components
            const i32 yOri = y - stride;
            const u32 upper_sample = digital_colors[yOri * ui::window_width + sample_x];
            const float upper_red = ((upper_sample & 0x00ff0000) >> 16) / 255.0f;
            const float upper_green = ((upper_sample & 0x0000ff00) >> 8) / 255.0f;
            const float upper_blue = (upper_sample & 0x000000ff) / 255.0f;
            const float upper_alpha = ((upper_sample & 0xff000000) >> 24) / 255.0f;

            // Resolve lower sample components
            const u32 lower_sample = digital_colors[y * ui::window_width + sample_x];
            const float lower_red = ((lower_sample & 0x00ff0000) >> 16) / 255.0f;
            const float lower_green = ((lower_sample & 0x0000ff00) >> 8) / 255.0f;
            const float lower_blue = (lower_sample & 0x000000ff) / 255.0f;
            const float lower_alpha = ((lower_sample & 0xff000000) >> 24) / 255.0f;

            // Blend each column between the reconstructed rows at either side of the stride
            for (u32 sample_y = yOri; sample_y < y; sample_y++)
            {
                const float t = static_cast<float>((sample_y - yOri)) / stride;
#ifdef RECONSTRUCT_BILINEAR
                const u32 s_red = static_cast<u32>(vmath::lerp(upper_red, lower_red, t) * 255.5f);
                const u32 s_green = static_cast<u32>(vmath::lerp(upper_green, lower_green, t) * 255.5f);
                const u32 s_blue = static_cast<u32>(vmath::lerp(upper_blue, lower_blue, t) * 255.5f);
                const u32 s_alpha = static_cast<u32>(vmath::lerp(upper_alpha, lower_alpha, t) * 255.5f);
                const u32 s = s_blue |
                             (s_green << 8) |
                             (s_red << 16) |
                             (s_alpha << 24);
#else
#ifdef RECONSTRUCT_BILINEAR_SQUARE
                const u32 s_red = static_cast<u32>(vmath::lerp(upper_red, lower_red, t) * 255.5f);
                const u32 s_green = static_cast<u32>(vmath::lerp(upper_green, lower_green, t) * 255.5f);
                const u32 s_blue = static_cast<u32>(vmath::lerp(upper_blue, lower_blue, t) * 255.5f);
                const u32 s_alpha = static_cast<u32>(vmath::lerp(upper_alpha, lower_alpha, t) * 255.5f);
                const u32 s = s_blue |
                             (s_green << 8) |
                             (s_red << 16) |
                             (s_alpha << 24);
#else
#ifdef RECONSTRUCT_NEAREST
                const u32 s = (t <= 0.5f) ? upper_sample : lower_sample;
#endif
#endif
#endif
                const u32 ndx = sample_y * ui::window_width + sample_x;
                camera::digital_colors[ndx] = s; // Using the same trick as above, since we know our pixel band is constant
            }
        }
    }

    void clear_patch(u32 xmin, u32 xmax, u32 ymin, u32 ymax)
    {
        u32 w = xmax - xmin;
        for (u32 y = ymin; y < ymax; y++)
        {
            const u32 ndx = y * ui::window_width + xmin;
            std::memset(digital_colors + ndx, 0, sizeof(u32) * w);
            std::memset(sensor_grid + ndx, 0, sizeof(sensel) * w);
            std::memset(filter_sum_grid + ndx, 0, sizeof(float) * w);
        }
    }

    // Initialize
    void init()
    {
        //assert(mem::tracing_arena != nullptr);
        sensor_grid = mem::allocate_tracing<sensel>(sensor_grid_footprint);
        digital_colors = mem::allocate_tracing<u32>(digital_colors_footprint);
        filter_sum_grid = mem::allocate_tracing<float>(filter_sum_grid_footprint);
        std::memset(filter_sum_grid, 0, filter_sum_grid_footprint);
    }
}