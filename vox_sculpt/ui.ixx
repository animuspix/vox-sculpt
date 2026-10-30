module;
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include "Resource.h"
#undef min
#undef max

export module ui;

import vox_ints;

// should integrate imgui here eventually

export namespace ui
{
    export HWND window_handle;
    export BITMAPINFO bitmap_info;

    constexpr u16 window_width = 1920;
    constexpr u16 window_height = 1080;
    constexpr u32 window_area = window_width * window_height;
    constexpr i16 image_centre_x = window_width / 2;
    constexpr i16 image_centre_y = window_height / 2;
    export bool window_setup(void* hInstance, int nCmdShow, void* wndProcSig, wchar_t* wndClassName, wchar_t* wndTitle)
    {
        WNDCLASSEXW window_class = {};
        window_class.cbSize = sizeof(WNDCLASSEXW);
        window_class.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
        window_class.lpfnWndProc = reinterpret_cast<WNDPROC>(wndProcSig);
        window_class.hInstance = static_cast<HINSTANCE>(hInstance);
        window_class.hIcon = LoadIcon(window_class.hInstance, MAKEINTRESOURCE(IDI_VOXSCULPT));
        window_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
        window_class.hbrBackground = static_cast<HBRUSH>(GetStockObject(LTGRAY_BRUSH));
        window_class.lpszMenuName = MAKEINTRESOURCEW(IDC_VOXSCULPT);
        window_class.lpszClassName = wndClassName;
        window_class.hIconSm = LoadIcon(window_class.hInstance, MAKEINTRESOURCE(IDI_SMALL));
        RegisterClassExW(&window_class);

        window_handle = CreateWindowW(wndClassName, wndTitle, WS_OVERLAPPEDWINDOW,
                                      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr,
                                      window_class.hInstance, nullptr);
        if (!window_handle)
        {
            return false;
        }

        ShowWindow(window_handle, nCmdShow);
        UpdateWindow(window_handle);

        bitmap_info = {};
        bitmap_info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bitmap_info.bmiHeader.biWidth = window_width;
        bitmap_info.bmiHeader.biHeight = window_height;
        bitmap_info.bmiHeader.biPlanes = 1;
        bitmap_info.bmiHeader.biBitCount = 32;
        bitmap_info.bmiHeader.biCompression = BI_RGB;
        return true;
    }
};