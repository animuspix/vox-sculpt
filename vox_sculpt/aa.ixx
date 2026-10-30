export module aa;
import vmath;
import vox_ints;

export namespace aa
{
	constexpr u32 samples_x = 8;
	constexpr u32 samples_y = 8;
	constexpr u32 max_samples = samples_x * samples_y;
	float blackman_harris_weight(vmath::vec<2> sample_xy)
	{
		// Transform coordinates to absolute pixel distances
		sample_xy = vmath::vabs(sample_xy - vmath::vec<2>(samples_x, samples_y));

		// Blackman-Harris alpha parameters
		// Parameter values taken from:
		// https://en.wikipedia.org/wiki/Window_function
		// (section "A list of window functions", subsection "Blackman-Harris window")
		const vmath::vec<2> alph0 = vmath::vec<2>(0.35875f);
		const vmath::vec<2> alph1 = vmath::vec<2>(0.48829f);
		const vmath::vec<2> alph2 = vmath::vec<2>(0.14128f);
		const vmath::vec<2> alph3 = vmath::vec<2>(0.01168f);

		// Core Blackman-Harris filter function
		vmath::vec<2> ratio = vmath::vec<2>(vmath::pi * sample_xy) / float(max_samples - 1);
		vmath::vec<2> filtv = alph0 - (alph1 * vmath::cos(2.0f * ratio)) +
			(alph2 * vmath::cos(4.0f * ratio)) -
			(alph3 * vmath::cos(6.0f * ratio));
		return (filtv.x() * filtv.y());
	}
	vmath::vec<2> supersample(float film_x, float film_y)
	{
		return vmath::vec<2>(film_x * samples_x, film_y * samples_y);
	}
	vmath::vec<2> jitter(float rand_u, float rand_v)
	{
		// Jitter inside the sampling grid
		const float x_jitter = (rand_u * aa::samples_x) - (aa::samples_x * 0.5f);
		const float y_jitter = (rand_v * aa::samples_y) - (aa::samples_y * 0.5f);
		return vmath::vec<2>(x_jitter, y_jitter);
	}
}