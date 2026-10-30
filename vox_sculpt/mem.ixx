export module mem;

import vox_ints;
import <cstddef>;
import <new>;

namespace mem
{
    constexpr u32 kilobyte = 1024;
    constexpr u64 megabyte = kilobyte * kilobyte;
    constexpr u64 gigabyte = megabyte * megabyte;
    typedef void* address;
    u8* tracing_arena; // Active memory for rendering operations, owned by render threads (no other arenas in use atm)
    u64 tracing_footprint; // Resolved at runtime
    address pool; // All the memory controlled by rtSoft
    u64 max_footprint;
    u64 alloc_offs; // Offset for dynamic allocations; allocations are thoretically possible but difficult at scale without
                    // compiler support
    export void init()
    {
        static_assert(sizeof(void*) == 8, "vox_sculpt needs at least 4GB of memory - 32bit builds will fail to allocate and crash early in startup");
        max_footprint = 0x100000000; // Arbitrary init size, since c++ doesn't seem to have great upfront allocation support and i kinda super ceebs continuing to
                                    // manage system footprints myself
        pool = ::operator new(static_cast<size_t>(max_footprint));
        tracing_arena = (u8*)pool;
        alloc_offs = 0;
    }
    export template<typename type_allocating>
    type_allocating* allocate_tracing(u32 num_bytes) // No alignment support atm, check with Athru as needed
    {
        type_allocating* ret_ptr = (type_allocating*)(tracing_arena + alloc_offs);
        alloc_offs += num_bytes;
        return ret_ptr;
    }
    export void deallocate_tracing(u32 num_bytes) // For short-term allocations needed by numerical arrays &c - most data should live until program exit
                                                  // Deallocates data from the end of our buffer, so every [deallocate] should have an earlier, matching [allocate] in the same scope
    {
        alloc_offs -= num_bytes;
    }
    export void deinit()
    {
        ::operator delete(pool);
    }
};