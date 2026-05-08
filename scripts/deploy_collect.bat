@echo off
setlocal enabledelayedexpansion

set PROJECT_ROOT=%~dp0..
set BUILD_DIR=%~dp0..\cmake-build-release-visual-studio
set DIST_DIR=%~dp0..\cmake-build-release-visual-studio\dist\pointworks
set PYTHON_EMBED=%~dp0..\3rdparty\python-embed

REM === Check required environment variables ===
if not defined QT_DIR (
    echo [ERROR] QT_DIR not set.
    echo.
    echo Please set it to your Qt 5.15 msvc2019_64 directory, e.g.:
    echo   set QT_DIR=D:\Qt\5.15.2\msvc2019_64
    exit /b 1
)
if not defined PCL_DIR (
    echo [ERROR] PCL_DIR not set.
    echo.
    echo Please set it to your PCL 1.12.1 root directory, e.g.:
    echo   set PCL_DIR=D:\LibCommunity\PCL 1.12.1
    exit /b 1
)

set QT_BIN=%QT_DIR%\bin
set PCL_BIN=%PCL_DIR%\bin
set VTK_BIN=%PCL_DIR%\3rdParty\VTK\bin
set FLANN_BIN=%PCL_DIR%\3rdParty\FLANN\bin
set QHULL_BIN=%PCL_DIR%\3rdParty\Qhull\bin

if not exist "%~dp0..\cmake-build-release-visual-studio\bin\pointworks.exe" (
    echo [ERROR] pointworks.exe not found in %~dp0..\cmake-build-release-visual-studio\bin
    echo Please run Release build first.
    exit /b 1
)

echo ============================================================
echo  PointWorks Deployment Collector
echo ============================================================
echo.

echo [1/8] Creating output directory...
if exist "%~dp0..\cmake-build-release-visual-studio\dist\pointworks" rmdir /s /q "%~dp0..\cmake-build-release-visual-studio\dist\pointworks"
mkdir "%~dp0..\cmake-build-release-visual-studio\dist\pointworks"
mkdir "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\platforms"
mkdir "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\styles"
mkdir "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\imageformats"
mkdir "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\scripts"
mkdir "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\scripts\examples"
mkdir "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\python"

echo [2/8] Copying exe and project DLLs...
copy "%~dp0..\cmake-build-release-visual-studio\bin\pointworks.exe" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\" >nul
for %%f in ("%~dp0..\cmake-build-release-visual-studio\bin\pw_*.dll") do copy "%%f" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\" >nul 2>&1

echo [3/8] Running windeployqt...
"%QT_BIN%\windeployqt.exe" --release --no-translations --no-opengl-sw "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\pointworks.exe"
if errorlevel 1 (
    echo [WARN] windeployqt failed
)

echo [4/8] Copying PCL DLLs (Release only)...
for %%f in ("%PCL_BIN%\pcl_*.dll") do (
    echo %%~nxf | findstr /e /i "d.dll" >nul 2>&1
    if errorlevel 1 (
        copy /Y "%%f" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\" >nul 2>&1
    )
)

echo [5/8] Copying VTK DLLs (Release only)...
for %%f in ("%VTK_BIN%\vtk*-9.1.dll") do (
    echo %%~nxf | findstr /e /i "d.dll" >nul 2>&1
    if errorlevel 1 (
        copy /Y "%%f" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\" >nul 2>&1
    )
)

echo [6/8] Copying 3rd-party DLLs...
copy /Y "%FLANN_BIN%\flann.dll" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\" >nul 2>&1
copy /Y "%FLANN_BIN%\flann_cpp.dll" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\" >nul 2>&1
copy /Y "%QHULL_BIN%\qhull_r.dll" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\" >nul 2>&1

echo [7/8] Copying embedded Python...
if not exist "%~dp0..\3rdparty\python-embed" (
    echo [WARN] Embedded Python not found at %~dp0..\3rdparty\python-embed
    echo        See README for setup instructions.
    goto :done
)
xcopy /E /Y /Q "%~dp0..\3rdparty\python-embed\*" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\python\" >nul
copy /Y "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\python\python39.dll" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\" >nul 2>&1
copy /Y "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\python\python3.dll" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\" >nul 2>&1
xcopy /E /Y /Q "%~dp0..\scripts\examples\*" "%~dp0..\cmake-build-release-visual-studio\dist\pointworks\scripts\examples\" >nul 2>&1

:done
echo [8/8] Done!
echo.
echo ============================================================
  Output: %DIST_DIR%
echo ============================================================
echo.
Next: run %DIST_DIR%\pointworks.exe to test
