[Setup]
AppName=MAGOS Radar Application
AppVersion=1.0
DefaultDirName={pf}\MAGOS
DefaultGroupName=MAGOS
OutputBaseFilename=MAGOS-Setup
Compression=lzma
SolidCompression=yes

[Files]
Source:"dist\MAGOS\*"; DestDir:"{app}"; Flags: recursesubdirs

[Icons]
Name:"{group}\MAGOS"; Filename:"{app}\MAGOS.exe"
