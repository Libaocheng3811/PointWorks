@echo off
setlocal

set BUILD_DIR=%~dp0..\cmake-build-release-visual-studio
set DIST_DIR=%~dp0..\cmake-build-release-visual-studio\dist\pointworks
set OUTPUT_DIR=%~dp0..\cmake-build-release-visual-studio\dist

if not exist "%DIST_DIR%\pointworks.exe" (
    echo [ERROR] pointworks.exe not found in %DIST_DIR%
    echo        Please run deploy_collect.bat first
    exit /b 1
)

set VERSION=0.9.0-beta
set ZIP_NAME=PointWorks-%VERSION%-Portable

echo Creating portable ZIP: %ZIP_NAME%.zip

powershell -NoProfile -ExecutionPolicy Bypass -Command "Compress-Archive -Path '%DIST_DIR%\*' -DestinationPath '%OUTPUT_DIR%\%ZIP_NAME%.zip' -Force"

if exist "%OUTPUT_DIR%\%ZIP_NAME%.zip" (
    echo Created: %OUTPUT_DIR%\%ZIP_NAME%.zip
) else (
    echo [ERROR] Failed to create ZIP
)

endlocal
