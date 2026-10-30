export module sampler;

// A generator for different ray-tracing and simulation distributions, including a CBRNG (Philox), a to-be-implemented QMC sequence (probably Sobol), and various useful hashes
// Sobol implementation is from Physically Based Rendering, 3rd Edition (Pharr, Jakob, Humphreys); Philox is described in the paper Parallel Random Numbers: As Easy as 1, 2, 3,
// available here,
// http://www.thesalmons.org/john/random123/papers/random123sc11.pdf
// and a provided API, available here:
// http://www.deshawresearch.com/downloads/download_random123.cgi/
// This implementation is one of the slightly worse ones - 4x32-7 (four chunks of 32-bit state, seven rounds iteration/step), trading off numerical reliabilty for easier implementation
// (no need for a custom software multiplier for 64-bit products vs 128-bit)

// Hash implementations are all random ones from the internet, sources given below
// :)
//////////////////////////////////////////////

import vox_ints;

export namespace sampler
{
    // Modified integer hash from iq, found in Nimitz's WebGL2 Quality Hashes Collection
    // (https://www.shadertoy.com/view/Xt3cDn)
    // Original from:
    // https://www.shadertoy.com/view/4tXyWN
    u32 ihashIII(u32 i)
    {
        i = 1103515245U * ((i >> 1U) ^ i);
        i = 1103515245U * (i ^ (i >> 3U));
        return i ^ (i >> 16);
    }

    // Lightly modified integer hash from Thomas Mueller, found here:
    // https://stackoverflow.com/questions/664014/what-integer-hash-function-are-good-that-accepts-an-integer-hash-key/12996028#12996028
    u32 tmhash(u32 x)
    {
        x = ((x >> 16) ^ x) * 0x45d9f3b;
        x = ((x >> 16) ^ x) * 0x45d9f3b;
        return (x >> 16) ^ x;
    }

    // Four-wide vector hash, modified from Nimitz's WebGL2 Quality Hashes Collection
    // (https://www.shadertoy.com/view/Xt3cDn)
    // Named as a mix between the basis hash (XXHash32-derived, https://github.com/Cyan4973/xxHash)
    // and the output transformation (MINSTD, http://random.mat.sbg.ac.at/results/karl/server/node4.html)
    void xxminstd32(u32 i, u32* out_x, u32* out_y, u32* out_z, u32* out_w)
    {
        const u32 PRIME32_2 = 2246822519U, PRIME32_3 = 3266489917U;
        const u32 PRIME32_4 = 668265263U, PRIME32_5 = 374761393U;
        u32 h32 = i + PRIME32_5;
        h32 = PRIME32_4 * ((h32 << 17) | (h32 >> (32 - 17))); //Initial testing suggests this line could be omitted for extra perf
        h32 = PRIME32_2 * (h32 ^ (h32 >> 15));
        h32 = PRIME32_3 * (h32 ^ (h32 >> 13));
        h32 ^= (h32 >> 16);

        // Outputs!
        // see http://random.mat.sbg.ac.at/results/karl/server/node4.html for details
        *out_x = h32;
        *out_y = h32 * 16807U;
        *out_z = h32 * 48271U;
        *out_w = h32 * 69621U;
    }

    // Wrapper for philox iteration + state management
    struct philox
    {
        u32 ctr[4];
        u64 key;
        void next(float* output)
        {
            for (u32 i = 0; i < 8; i++)
            {
                // Update state
                constexpr u64 m0 = 0xCD9E8D57; // Multiplier from the Random123 paper, page 7
                constexpr u64 m1 = 0xD2511F53; // Multiplier from the Random123 paper, page 7
                u64 mul0 = ctr[0] * m0;
                u64 mul1 = ctr[2] * m1;
                u64 mul1_hi = (mul1 << 32);
                ctr[0] = (mul1 >> 32) ^ ctr[1] ^ ((key << 32) >> 32);
                ctr[1] = (mul1 << 32) >> 32;
                ctr[2] = (mul0 >> 32) ^ ctr[2] ^ (key >> 32);
                ctr[3] = (mul0 << 32) >> 32;

                // Update key
                constexpr u64 bump = (0x9E3779B9 | ((u64)0xBB67AE85 << 32));
                key += bump;
            }

            // Pass updated state back to the callsite
            output[0] = ctr[0] / 4294967295.0f;
            output[1] = ctr[1] / 4294967295.0f;
            output[2] = ctr[2] / 4294967295.0f;
            output[3] = ctr[3] / 4294967295.0f;
        }

        // Initialization assumes one seed/tile
        void init(u8 tileNdx)
        {
            xxminstd32(tileNdx, ctr, ctr + 1, ctr + 2, ctr + 3); // Seed state by hashing indices with xxminstd32 (see above)
            key = (((u64)ihashIII(tileNdx) << 32) |
                   ((u64)tmhash(tileNdx))); // Partially seed keys with a different hash (not Wang or xxminstd32)
        }
    };
};