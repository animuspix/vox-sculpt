export module parallel;

import ui;
import platform;
import sampler;
import mem;
import vmath;
import vox_ints;
import <stdio.h>;
import <cmath>;
import <cstring>;
import <atomic>;
import <memory>;
import <thread>;

//#define PARALLEL_DBG
#ifdef PARALLEL_DBG
#pragma optimize("", off)
#endif

export namespace parallel
{
    u16 numTilesX = 0;
    u16 numTilesY = 0;
    u16 numTiles = 0;
    platform::threads::osThread<platform::threads::thread_meta::TILE>* tiles = nullptr;
    sampler::philox* rand_streams = nullptr; // One PRNG stream/tile
    char** logs = nullptr; // Debug messages, printable after tasks have finished and threads are idle
    u32* log_lengths = nullptr; // Amount of valid text per-log; reset after printing
    void init()
    {
        // Find maximum tile count for the current platform
        numTiles = std::thread::hardware_concurrency() - 2;
        const float root = std::sqrt(static_cast<float>(numTiles));
        if (root - int(root) > 0)
        {
            // X is the root of the previous perfect square
            numTilesX = static_cast<u16>(std::floor(root));

            // We want our threads to tile as evenly as possible;
            // approximate tiles_y by dividing out the total number
            // of tiles by the number we assigned for X
            // We could go further, but this works in most cases and
            // leaves threads around for doing other things on the
            // user's pc when it doesn't; that feels better than a
            // solution that works in "all" cases but generates more
            // threads than the hardware can run independantly,
            // causing some to timeslice
            numTilesY = numTiles / numTilesX;

            // Num-tiles can diverge from the actual number of tiles in this case, due to rounding issues
            // (we can't claim fractions of a core)
            // Update num-tiles so it always matches numTilesX * numtilesY
            numTiles = numTilesX * numTilesY;
        }
        else
        {
            numTilesX = static_cast<u16>(root);
            numTilesY = static_cast<u16>(root);
        }

        // Allocat thread tiles
        tiles = mem::allocate_tracing<platform::threads::osThread<platform::threads::thread_meta::TILE>>(sizeof(platform::threads::osThread<platform::threads::thread_meta::TILE>) * numTiles);
        for (u16 i = 0; i < numTiles; i++)
        {
            std::construct_at(tiles + i);
        }

        // Allocate PRNG streams
        rand_streams = mem::allocate_tracing<sampler::philox>(sizeof(sampler::philox) * numTiles);

        // Allocate log destinations
        // (can probably optimize to subdivide one huge char* instead of lots of cache-unfriendly pointers)
        logs = mem::allocate_tracing<char*>(numTiles);
        for (u16 i = 0; i < numTiles; i++)
        {
            logs[i] = mem::allocate_tracing<char>(4 * 1024 * 1024);
            std::memset(logs[i], 0, 4 * 1024 * 1024);
        }

        // Allocate log-length tags
        // Each tag gives a length of valid/printable text for each log
        log_lengths = mem::allocate_tracing<u32>(numTiles * sizeof(u32));

        // Initialize random-number-generators, thread-state fences, and worker threads
        // Each tile gets up to 4MB of debug output
        for (u8 i = 0; i < (u8)numTiles; i++)
        {
            rand_streams[i].init(i);
        }
        platform::threads::osInitBatchProcessors(tiles, numTiles);
    }

    // Work centre can dispatch anything that takes x/y tile counts + a tile index
    void launch(void(*work)(u16 num_tiles_x, u16 num_tiles_y, u16 tile_ndx))
    {
        // Draw workers are constantly integrating the scene
        platform::threads::osAssignAndWakeBatchProcessors(work, tiles, static_cast<u8>(numTilesX), static_cast<u8>(numTilesY), numTiles);
    }
    char* get_log_dst(u8 tile_ndx)
    {
        return logs[tile_ndx] + log_lengths[tile_ndx];
    }
    template<typename...input_types>
    void append_tile_log(u8 tile_ndx, const char* str, input_types...inputs)
    {
        char* dst = get_log_dst(tile_ndx);
        u16 dLen = 0;
        while (str[dLen] != '\0')
        {
            dLen++;
        }
        sprintf_s(dst, dLen, str, inputs...);
        log_lengths[tile_ndx] += dLen;
    }
    void stop_work()
    {
        platform::threads::osTerminateBatchProcessors(tiles, numTiles);
    }
};

#ifdef PARALLEL_DBG
#pragma optimize("", on)
#endif
