# librokae-v0.2.8 Environment / Migration Notes

Generated on 2026-06-24 from `D:\roake\sdk`.

This note only covers the core project:

- Source project: `D:\roake\sdk\librokae-v0.2.8\librokae-v0.2.8`
- Clean migration folder: `D:\roake\sdk\librokae-v0.2.8_clean`

Other folders in the original SDK directory are not required for this core project and were intentionally left out of the clean package.

## Project Summary

- Project name: `librokae`
- Project version: `0.2.8`
- Language: C++
- C++ standard: C++17
- Build system: CMake
- Minimum CMake version: `3.12`
- Main platform target: Windows x64 / Linux x86_64
- Main optional CMake flag: `XCORE_USE_XMATE_MODEL`, currently `OFF`

README compatibility notes:

- Controller: xCore controller version `v1.7+`
- Windows: Windows 10, MSVC `14.1+`, x86_64, C++
- Ubuntu: 18.04 / 20.04 / 22.04, build-essential, x86_64, C++

## Current Machine Environment

- OS: Windows 10 Home China, WindowsVersion `2009`
- Visual Studio: Visual Studio Community 2022
- Visual Studio version: `17.5.1`
- Visual Studio installation version: `17.5.33424.131`
- Visual Studio path: `C:\Program Files\Microsoft Visual Studio\2022\Community`
- MSVC toolchain used by existing builds: `14.35.32215`
- MSVC compiler version: `19.35.32215`
- CMake bundled with Visual Studio: `3.25.1-msvc1`
- Ninja bundled with Visual Studio: `1.11.0`

Important: in normal PowerShell on this machine, `cmake`, `ninja`, `cl`, and `msbuild` are not on `PATH`. They are available through Visual Studio, the Visual Studio Developer Prompt, or Visual Studio's CMake integration.

## Clean Folder Layout

The cleaned folder removes the repeated nesting. Open this directory directly:

```text
D:\roake\sdk\librokae-v0.2.8_clean
```

Main contents:

- `CMakeLists.txt`
- `CMakeSettings.json`
- `README.md`
- `CHANGELOG.md`
- `example\cpp` - example source files and public headers
- `external\Eigen` - vendored header-only Eigen dependency
- `lib` - prebuilt SDK libraries for Windows and Linux
- `doc\doxygen` - Doxygen configuration/templates

Removed from the clean folder:

- `.vs`
- `out`
- `build`
- generated `x64`, `Debug`, and `Release` build output folders outside SDK library folders
- generated files such as `.exe`, `.pdb`, `.obj`, `.ilk`, `.exp`, `.idb`, `.tlog`, `.sdf`, `.user`
- old archives and unrelated SDK/demo folders

## Required Bundled Libraries

These library files are part of the SDK and should be kept:

- `lib\Windows\cpp\Debug\32bit\RokaeSDK.lib`
- `lib\Windows\cpp\Debug\64bit\RokaeSDK.lib`
- `lib\Windows\cpp\Release\32bit\RokaeSDK.lib`
- `lib\Windows\cpp\Release\64bit\RokaeSDK.lib`
- `lib\Linux\cpp\x86_64\libRokaeSDK.a`

Although `Debug` and `Release` are directory names, these are not disposable build folders in this case. They contain the SDK's prebuilt libraries and must be copied.

## Recommended Setup On Another Windows PC

1. Install Visual Studio 2022 Community or newer.
2. Select the workload `Desktop development with C++`.
3. Ensure MSVC v143 x64/x86 build tools are installed.
4. Ensure Windows 10 SDK is installed.
5. Ensure CMake and Ninja are installed, either as Visual Studio components or standalone tools.
6. Copy `D:\roake\sdk\librokae-v0.2.8_clean` to the new PC.
7. Open `librokae-v0.2.8_clean` directly as a CMake folder in Visual Studio.

## Build Notes

Visual Studio CMake settings in `CMakeSettings.json`:

- `x64-Debug`
  - Generator: `Ninja`
  - Configuration type: `Debug`
  - Inherits: `msvc_x64_x64`
- `x64-Release`
  - Generator: `Ninja`
  - Configuration type: `RelWithDebInfo`
  - Inherits: `msvc_x64_x64`

Existing build-cache evidence from the original machine:

- CMake generator used: `Ninja` and `Visual Studio 17 2022`
- Compiler path used:
  `C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.35.32215\bin\Hostx64\x64\cl.exe`
- Linker path used:
  `C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.35.32215\bin\Hostx64\x64\link.exe`

## Linux Notes

For Linux x86_64, the SDK includes:

- `lib\Linux\cpp\x86_64\libRokaeSDK.a`

README states supported Ubuntu versions:

- Ubuntu 18.04
- Ubuntu 20.04
- Ubuntu 22.04

Install typical build tools:

```bash
sudo apt install cmake g++
```

