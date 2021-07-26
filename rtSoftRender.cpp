// rtSoftRender.cpp : Defines the entry point for the application.
//

#include "ui.h"
#include "camera.h"
#include <windows.h>
#undef min
#undef max
#include "resource.h"
#include "parallel.h"
#include "io.h"
#include "geometry.h"

#define MAX_LOADSTRING 100

// Global Variables:
                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name
decltype(camera::digital_colors) backBuf; // Back-buffer for window output (separated from live image output to allow asynchronous rendering/presentation)

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // Initialize global strings
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_RTSOFTRENDER, szWindowClass, MAX_LOADSTRING);

    // Initialize rendering systems
    mem::init();
    camera::init();
    tracing::init();
    geometry::init();

    // Launch drawing work
    parallel::launch();

    // Create the application window
    if (!ui::window_setup(hInstance, nCmdShow, WndProc, szWindowClass, szTitle)) return FALSE;

    // Load window/desktop renderer
    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_RTSOFTRENDER));

    // Main message loop:
    uint64_t frameCtr = 0;
    double frameTimeBuffer = 0.0;
    double avgFrameTime = 0.0;
    double last_frame_time = 0.0;
    auto _now = std::chrono::steady_clock::now();
#ifdef PROFILE
    constexpr uint32_t dbgOutLen = 64;
    char dbgOut[dbgOutLen];
    memset(dbgOut, 0x0, dbgOutLen);
#endif
    MSG msg;
    backBuf = mem::allocate_tracing<RGBQUAD>(camera::digital_colors_footprint);
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        // Update colors for rendering
        if (io::present_switch.load())
        {
            // Double-buffer output into "backBuf" so we can draw frames without thinking about messages
            memcpy(backBuf, camera::digital_colors, camera::digital_colors_footprint);

            // Loaded output bits into the backbuffer, so start allowing drawing again on each thread/tile
            io::present_switch.store(false);
            for (uint32_t i = 0; i < parallel::numTiles; i++) parallel::drawFinished[i].store(false);

            // Update frame statistics
#ifdef PROFILE
            frameCtr++;
            auto t = std::chrono::steady_clock::now();
            auto dt = (t - _now).count() / 1000000000.0;
            last_frame_time = dt; // Clock tick count is in nanoseconds by default, convert down to seconds
            frameTimeBuffer += dt;
            avgFrameTime = frameTimeBuffer / frameCtr;
            _now = t;
            sprintf_s(dbgOut, "last frame time %f \n", last_frame_time);
            OutputDebugStringA(dbgOut);
            memset(dbgOut, 0x0, dbgOutLen);
#endif
        }

        // Present new colors on window wake/message
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            InvalidateRect(ui::hWnd, NULL, false);
        }
    }

    // Clean-up, exit the program :)
    parallel::stop_work();
    mem::deinit();
    return (int) msg.wParam;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style          = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_RTSOFTRENDER));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_RTSOFTRENDER);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
    return RegisterClassExW(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   //hInst = hInstance; // Store instance handle in our global variable

   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE: Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            // Parse the menu selections:
            switch (wmId)
            {
            case IDM_EXIT:
                DestroyWindow(hWnd);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
        break;
    case WM_PAINT:
        {
            // Ready to present, so send output bits off to the window surface
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);
            uint32_t test = SetDIBitsToDevice(hdc, 0, 0, ui::window_width, ui::window_height, 0, 0, 0, ui::window_height, camera::digital_colors, &ui::bmi, DIB_RGB_COLORS);
            EndPaint(hWnd, &ps);
        }
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}