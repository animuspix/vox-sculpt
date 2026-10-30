module;
#include <Windows.h>
#undef min
#undef max

export module platform;

import vox_ints;
import mem;
import <atomic>;
import <memory>;
import <type_traits>;
import <tuple>;

// Thread workload prototypes
// Different types of system thread can have different types of work
// Declared up here to avoid implicitly exporting these to other modules
// (atm exporting members but not the namespace will make the members unreachable - I think I tried that before)
void(*worker_fn_prototype)(); // Workers
void(*tile_fn_prototype)(u16, u16, u16); // Work-group width, work-group height, work-group index


export namespace platform
{
    // Foward-declare _everything_ to keep implementations away from modules
    namespace threads
    {
        // Architectural details for the threading system used by vox_sculpt
        // Considering moving these to their own module
        // (especially if I eventually decide to support graphics accelerators)
        namespace thread_meta
        {
            // Types of async thread supported
            enum THREAD_TYPES
            {
                WORKER, // A thread associated with a job-stream, or a single task
                TILE // A tile thread, designed for batch processing within a workgroup
            };

            // Work-item prototypes for each thread-type
            typedef decltype(worker_fn_prototype) worker_thrd_fn;
            typedef decltype(tile_fn_prototype) tile_thrd_fn;

            // Parameter layouts for each thread-type (worker threads take no parameters for now)
            struct worker_params {};
            struct tile_params
            {
                u8 workgroup_width;
                u8 workgroup_height;
                u16 workgroup_index;
            };
        };

        enum thread_state
        {
            WORKING = 0x00000000,
            SLEEPING = 0xffffffff
        };

        class osThreadGeneric
        {
            public:
            osThreadGeneric(): handle(nullptr),
                               messaging(nullptr),
                               state(nullptr),
                               cvar(nullptr),
                               thread_guard(nullptr),
                               thread_openness(nullptr),
                               initialized(false) {}; // Generic thread types cannot be freely created or destroyed, only upcasted from regular thread types (to allow for flexible code without
                                                            // explicit templates everywhere)
            ~osThreadGeneric(){};
            void* handle; // Associated with a Win32 HANDLE in per-platform code
            std::atomic<long>* messaging; // Atomic status, used for communicating with main/other threads
            std::atomic<long>* state; // Thread system state: running, sleeping, or an error code
                                // Should be read by main and written by [this], except on thread launch

            // Checks if the specified thread has finished, and returns immediately if it hasn't
            //void osWaitNonblocking(osThreadGeneric* thread);

            // Test if a thread has been initialized already
            bool osCheckInitialized() { return initialized; }

            // Batch processors (TILEs) use a single statically-declared cvar to control execution, and a shared one for independant threads (WORKERs)
            void* cvar;

            // Implemented as a critical section (kind-of a user-space mutex) under Windows
            void* thread_guard;

            // Zero for closed, one for open
            // This enables efficiently terminating individual threads, while allowing regular stack unwinding & memory reclamation
            std::atomic<long>* thread_openness;

            protected:
            bool initialized;
        };

        template<thread_meta::THREAD_TYPES type>
        class osThread : public osThreadGeneric
        {
            public:
                osThread() {};

                //typedef std::conditional<type == WORKER, decltype(worker_fn_prototype), decltype(tile_fn_prototype)>::type work_type; // std::conditional seems to be buggy with modules, using tuples for now
                std::tuple<thread_meta::worker_thrd_fn, thread_meta::tile_thrd_fn> work_types;
                typedef std::remove_reference<decltype(std::get<type == thread_meta::WORKER ? 0 : 1>(work_types))>::type work_type;
                work_type next_task = nullptr; // Pointer to next workload to execute (no jobs for now, so just one task/thread)

                std::tuple<thread_meta::worker_params, thread_meta::tile_params> work_paramsets;
                typedef std::remove_reference<decltype(std::get<type == thread_meta::WORKER ? 0 : 1>(work_paramsets))>::type work_params;
                work_params next_task_params = {}; // Cached parameters for each task invocation
        };

        // Find tile count for the current cpu (at runtime)
        // We assume that we flood the cpu with tile work, and any independant worker threads are timesliced in (since they'll most likely be i/o tasks that we
        // don't super mind running behind slightly)
        // Update arguments for batched threads, then wake them
        void osAssignAndWakeBatchProcessors(thread_meta::tile_thrd_fn fn,
                                            osThread<thread_meta::TILE>* workgroup, u8 workgroup_width, u8 workgroup_height, u16 workgroup_area);

        // Update arguments for the given worker thread, then wake it
        void osAssignAndWakeWorker(thread_meta::worker_thrd_fn fn, osThread<thread_meta::WORKER>* worker);

        // Create & initialize batch processors
        void osInitBatchProcessors(osThread<thread_meta::TILE>* workgroup, u16 workgroup_area);

        // Create & initialize a single worker thread
        void osInitWorker(osThread<thread_meta::WORKER>* worker);

        // Wake & terminate batched threads
        void osTerminateBatchProcessors(osThread<thread_meta::TILE>* workgroup, u16 workgroup_area);

        // Wake & terminate a worker thread
        void osTerminateWorker(osThread<thread_meta::WORKER>* worker);

    };
};

// Thread workload prototypes
template<typename callable_t>
concept thread_task_type = std::is_same_v<callable_t, platform::threads::thread_meta::worker_thrd_fn> ||
                           std::is_same_v<callable_t, platform::threads::thread_meta::tile_thrd_fn>;

template<thread_task_type task_type>
struct osThreadControls
{
    PCONDITION_VARIABLE _cvar;
    PCRITICAL_SECTION _crit_section;
    task_type* _next_task;
    void* _work_params;
    std::atomic<long>* _state;
    platform::threads::thread_meta::THREAD_TYPES _type;
    std::atomic<long>* _thread_openness;
    bool _work_pending;
};

osThreadControls<platform::threads::thread_meta::tile_thrd_fn>* batched_thrd_controls = nullptr;

#define DBG_THREAD_MAIN
#ifdef DBG_THREAD_MAIN
#pragma optimize ("", off)
#endif
DWORD WINAPI thread_main_tiles(void* ctrl)
{
    auto* tile_controls = static_cast<osThreadControls<platform::threads::thread_meta::tile_thrd_fn>*>(ctrl);
    for (;;)
    {
        EnterCriticalSection(tile_controls->_crit_section);
        while (tile_controls->_thread_openness->load() && !tile_controls->_work_pending)
        {
            if (!SleepConditionVariableCS(tile_controls->_cvar, tile_controls->_crit_section, INFINITE))
            {
                LeaveCriticalSection(tile_controls->_crit_section);
                return GetLastError();
            }
        }

        if (!tile_controls->_thread_openness->load())
        {
            LeaveCriticalSection(tile_controls->_crit_section);
            break;
        }

        tile_controls->_work_pending = false;
        const auto task = *tile_controls->_next_task;
        const auto params = *static_cast<platform::threads::thread_meta::tile_params*>(tile_controls->_work_params);
        LeaveCriticalSection(tile_controls->_crit_section);

        tile_controls->_state->store(platform::threads::WORKING);
        task(params.workgroup_width, params.workgroup_height, params.workgroup_index);
        tile_controls->_state->store(platform::threads::SLEEPING);
    }
    return 0;
}
#ifdef DBG_THREAD_MAIN
#pragma optimize ("", on)
#endif

void platform::threads::osInitBatchProcessors(platform::threads::osThread<platform::threads::thread_meta::TILE>* workgroup,
                                              u16 workgroup_area)
{
    if (batched_thrd_controls == nullptr)
    {
        batched_thrd_controls = mem::allocate_tracing<osThreadControls<platform::threads::thread_meta::tile_thrd_fn>>(
            sizeof(osThreadControls<platform::threads::thread_meta::tile_thrd_fn>) * workgroup_area);
    }

    for (u16 i = 0; i < workgroup_area; i++)
    {
        if (!workgroup[i].osCheckInitialized())
        {
            workgroup[i].cvar = mem::allocate_tracing<CONDITION_VARIABLE>(sizeof(CONDITION_VARIABLE));
            InitializeConditionVariable(static_cast<PCONDITION_VARIABLE>(workgroup[i].cvar));
            workgroup[i].thread_guard = mem::allocate_tracing<CRITICAL_SECTION>(sizeof(CRITICAL_SECTION));
            InitializeCriticalSection(static_cast<LPCRITICAL_SECTION>(workgroup[i].thread_guard));

            workgroup[i].messaging = std::construct_at(mem::allocate_tracing<std::atomic<long>>(sizeof(std::atomic<long>)));
            workgroup[i].messaging->store(0);

            workgroup[i].state = std::construct_at(mem::allocate_tracing<std::atomic<long>>(sizeof(std::atomic<long>)));
            workgroup[i].state->store(0);

            workgroup[i].thread_openness = std::construct_at(mem::allocate_tracing<std::atomic<long>>(sizeof(std::atomic<long>)));
            workgroup[i].thread_openness->store(1);

            osThreadControls<platform::threads::thread_meta::tile_thrd_fn>& tile_controls = batched_thrd_controls[i];
            tile_controls._cvar = static_cast<PCONDITION_VARIABLE>(workgroup[i].cvar);
            tile_controls._crit_section = static_cast<PCRITICAL_SECTION>(workgroup[i].thread_guard);
            tile_controls._next_task = &workgroup[i].next_task;
            tile_controls._work_params = &workgroup[i].next_task_params;
            tile_controls._state = workgroup[i].state;
            tile_controls._type = platform::threads::thread_meta::TILE;
            tile_controls._thread_openness = workgroup[i].thread_openness;
            tile_controls._work_pending = false;
        }
        else
        {
            DebugBreak();
        }
    }

    for (u16 i = 0; i < workgroup_area; i++)
    {
        workgroup[i].handle = CreateThread(nullptr, 0xffff, thread_main_tiles, &batched_thrd_controls[i], 0, nullptr);
    }
}

void platform::threads::osInitWorker(platform::threads::osThread<platform::threads::thread_meta::WORKER>* worker)
{
    (void)worker;
}

void platform::threads::osAssignAndWakeBatchProcessors(platform::threads::thread_meta::tile_thrd_fn fn,
                                                       platform::threads::osThread<platform::threads::thread_meta::TILE>* workgroup,
                                                       u8 workgroup_width,
                                                       u8 workgroup_height,
                                                       u16 workgroup_area)
{
    for (u16 i = 0; i < workgroup_area; i++)
    {
        EnterCriticalSection(static_cast<PCRITICAL_SECTION>(workgroup[i].thread_guard));
        workgroup[i].next_task = fn;
        workgroup[i].next_task_params = { workgroup_width, workgroup_height, i };
        batched_thrd_controls[i]._work_pending = true;
        WakeConditionVariable(static_cast<PCONDITION_VARIABLE>(workgroup[i].cvar));
        LeaveCriticalSection(static_cast<PCRITICAL_SECTION>(workgroup[i].thread_guard));
    }
}

void platform::threads::osAssignAndWakeWorker(platform::threads::thread_meta::worker_thrd_fn fn,
                                              platform::threads::osThread<platform::threads::thread_meta::WORKER>* worker)
{
    worker->next_task = fn;
    WakeConditionVariable(static_cast<PCONDITION_VARIABLE>(worker->cvar));
}

void platform::threads::osTerminateBatchProcessors(
    platform::threads::osThread<platform::threads::thread_meta::TILE>* workgroup, u16 workgroup_area)
{
    for (u16 i = 0; i < workgroup_area; i++)
    {
        EnterCriticalSection(static_cast<PCRITICAL_SECTION>(workgroup[i].thread_guard));
        workgroup[i].thread_openness->fetch_sub(1);
        WakeConditionVariable(static_cast<PCONDITION_VARIABLE>(workgroup[i].cvar));
        LeaveCriticalSection(static_cast<PCRITICAL_SECTION>(workgroup[i].thread_guard));
    }

    for (u16 i = 0; i < workgroup_area; i++)
    {
        WaitForSingleObject(static_cast<HANDLE>(workgroup[i].handle), INFINITE);
        DeleteCriticalSection(static_cast<PCRITICAL_SECTION>(workgroup[i].thread_guard));
        CloseHandle(static_cast<HANDLE>(workgroup[i].handle));
    }
}

void platform::threads::osTerminateWorker(
    platform::threads::osThread<platform::threads::thread_meta::WORKER>* worker)
{
    worker->thread_openness->fetch_sub(1);
    WakeConditionVariable(static_cast<PCONDITION_VARIABLE>(worker->cvar));
    WaitForSingleObject(static_cast<HANDLE>(worker->handle), INFINITE);
    TerminateThread(static_cast<HANDLE>(worker->handle), 0);
    DeleteCriticalSection(static_cast<PCRITICAL_SECTION>(worker->thread_guard));
}
