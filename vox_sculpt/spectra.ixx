export module spectra;
import vox_ints;
import vmath;
import <cmath>;

export namespace spectra
{
    // Super basic sky model - sharp blue fading to orange as y approaches zero
    // Desmos visualization
    // https://www.desmos.com/calculator/7qqnkr1uqx
    float sky(float rho, float y)
    {
        const float blue = vmath::clamp(vmath::gaussian(rho, 0.25f, 0.18f, 0.1f, -0.75f), 0.0f, 1.0f);
        const float orange = vmath::clamp(vmath::gaussian(rho, 0.25f, 0.7f, 0.1f, -0.75f), 0.0f, 1.0f);
        return vmath::lerp(blue, orange, std::abs(y));
    }

    // Custom film/sensor response curve, kinda follows the references in Zucconi's diffraction tutorial but heavily iterated to give a more film-like
    // spectrum; Zucconi's tutorial is here
    // https://www.alanzucconi.com/2017/07/15/improving-the-rainbow/,
    // aand my Desmos workspace for the curves themselves is here:
    // https://www.desmos.com/calculator/zm9jwr0ngk
    vmath::vec<3> film(float rho)
    {
        return vmath::vec<3>(vmath::max(vmath::quadratic(rho, 4.0f, 0.6f, 0.2f, true), 0.0f) +
                             vmath::max(vmath::quadratic(rho, 4.0f, 3.0f, 1.0f, true), 0.0f),
                             vmath::max(vmath::gaussian(rho, 1.0f, 0.5f, 0.2f, 0.05f), 0.0f),
                             vmath::max(vmath::gaussian(rho, 1.0f, 0.0f, 0.55f, 0.2f) *
                                         vmath::quadratic(rho / 0.4f, 1.0f, -0.6f / 0.4f, -2.3f, false) *
                                         vmath::quadratic(rho, 1.0f, 0.95f, 0.0f, false) + 0.1f, 0.0f));
    }

    // Generic diffuse surface colors until I get imgui working
    const float placeholder_spd(vmath::vec<4> coords_and_rho) // [xyz], w (wavelength)
    {
        return vmath::gaussian(coords_and_rho.w(), 2.7f, 0.8f, 0.2f, 1.7f); // Debug red :p
    }

    // Stratified spectral representation for camera sampling
    export struct spectral_buckets
    {
        static constexpr u32 num_buckets = 16;
        static constexpr float interval_size = 1.0f / num_buckets;
        u32 last_bucket = 0;
        float buckets[num_buckets] = {};
        void init()
        {
            // Random bucket selection by default
            for (u32 i = 0; i < num_buckets; i++)
            {
                buckets[i] = 0.0f;
            }
        }
        float draw_sample(float u, float v)
        {
            // Try to select a frequency from a curve skewed towards colors reflected most by the environment
            // This should work for most scenes
            for (u32 n = 0; n < num_buckets; n++)
            {
                if (u < buckets[n]) // Buckets with higher sensitivities will naturally pass this test more often over time
                {
                    last_bucket = n; // Record the selected bucket so we can update it with a weight after tracing
                    return (n * interval_size) + (v * interval_size); // Return a random offset within the selected bucket
                }
            }

            // The surface is matte black; all frequencies reflected equally poorly
            // Since no frequencies are particularly bright, just return one at random
            last_bucket = static_cast<u32>(std::floor(u * num_buckets));
            return (last_bucket * interval_size) + (v * interval_size);
        }
        void update(float weight)
        {
            buckets[last_bucket] = weight; // Any noise from using per-sample weights seems to be cancelled out during filtering, and
                                           // naively smoothing here (e.g. by using a weight / num_samples box filter) seems to damage the
                                           // importance function (by skewing weights down) more than it helps account for natural spectral
                                           // variance
        }
    };
};
