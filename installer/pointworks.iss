; ============================================================
;  PointWorks - Inno Setup 安装脚本
;  使用方法:
;    1. 安装 Inno Setup 6.x (https://jrsoftware.org/isdl.php)
;    2. 先运行 scripts/deploy_collect.bat 收集依赖到 dist/pointworks/
;    3. 用 Inno Setup Compiler 打开此文件并编译
; ============================================================

#define MyAppName "PointWorks"
#define MyAppVersion "1.0.0"
#define MyAppPublisher "PointWorks"
#define MyAppExeName "pointworks.exe"
#define MyAppURL ""
; 路径通过环境变量传入，或用命令行 /D 覆盖，例如:
;   set PROJECT_ROOT=C:\my\project
;   set BUILD_OUTPUT=C:\my\project\cmake-build-release-visual-studio
;   "C:\Program Files (x86)\Inno Setup 6\ISCC.exe" /DProjectRoot="C:\my\project" /DBuildOutput="C:\my\project\cmake-build-release-visual-studio" installer\pointworks.iss
#define ProjectRoot GetEnv("PROJECT_ROOT")
#define SourceDir GetEnv("BUILD_OUTPUT") + "\dist\pointworks"

[Setup]
AppId={{A1B2C3D4-E5F6-7890-ABCD-EF1234567890}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={autopf}\{#MyAppName}
DefaultGroupName={#MyAppName}
OutputDir={#ProjectRoot}\cmake-build-release-visual-studio\dist
OutputBaseFilename=PointWorks-Setup-{#MyAppVersion}
Compression=lzma2/ultra64
SolidCompression=yes
SetupIconFile={#ProjectRoot}\src\resources\logo\logo.ico
WizardStyle=modern
PrivilegesRequired=lowest
; 64-bit 安装
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "chinese"; MessagesFile: "compiler:Languages\ChineseSimplified.isl"

[Tasks]
Name: "desktopicon"; Description: "Create desktop shortcut"; GroupDescription: "Shortcuts:"
Name: "startmenu"; Description: "Create Start Menu shortcut"; GroupDescription: "Shortcuts:"

[Files]
; 主程序
Source: "{#SourceDir}\{#MyAppExeName}"; DestDir: "{app}"; Flags: ignoreversion
; 项目 DLL
Source: "{#SourceDir}\ct_*.dll"; DestDir: "{app}"; Flags: ignoreversion
; Python 运行时（exe 同级）
Source: "{#SourceDir}\python3*.dll"; DestDir: "{app}"; Flags: ignoreversion
; Qt DLL
Source: "{#SourceDir}\Qt5*.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\libEGL.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\libGLESv2.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\D3Dcompiler_47.dll"; DestDir: "{app}"; Flags: ignoreversion
; Qt 插件
Source: "{#SourceDir}\platforms\*"; DestDir: "{app}\platforms"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#SourceDir}\styles\*"; DestDir: "{app}\styles"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#SourceDir}\imageformats\*"; DestDir: "{app}\imageformats"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#SourceDir}\iconengines\*"; DestDir: "{app}\iconengines"; Flags: ignoreversion recursesubdirs createallsubdirs
; VTK DLL
Source: "{#SourceDir}\vtk*.dll"; DestDir: "{app}"; Flags: ignoreversion
; PCL DLL
Source: "{#SourceDir}\pcl_*.dll"; DestDir: "{app}"; Flags: ignoreversion
; 第三方 DLL
Source: "{#SourceDir}\flann*.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\qhull*.dll"; DestDir: "{app}"; Flags: ignoreversion
; Python 嵌入式发行版
Source: "{#SourceDir}\python\*"; DestDir: "{app}\python"; Flags: ignoreversion recursesubdirs createallsubdirs
; 示例脚本
Source: "{#SourceDir}\scripts\*"; DestDir: "{app}\scripts"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\{#MyAppExeName}"; Tasks: desktopicon
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: startmenu

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "Launch PointWorks"; Flags: nowait postinstall skipifsilent

[UninstallDelete]
Type: filesandordirs; Name: "{app}"
