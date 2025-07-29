# MAGOS Radar Application

This project contains a PyQt-based GUI for interacting with the MAGOS radar.

## Requirements

- Python 3.9+
- `requests`
- Either `PyQt6` and `PyQt6-WebEngine` or `PyQt5` and `PyQtWebEngine`
- `pyinstaller` (for building the executable)
- Optionally, [Inno Setup](https://jrsoftware.org/isinfo.php) to create a Windows installer

Install dependencies with pip:
```bash
pip install requests PyQt6 PyQt6-WebEngine pyinstaller
```
Or, for PyQt5:
```bash
pip install PyQt5 PyQtWebEngine pyinstaller
```

## Building a Stand-alone Executable

Run PyInstaller with the provided spec file:
```bash
pyinstaller magos.spec
```
The resulting standalone executable will be located in the `dist/` folder.

## Creating a Windows Installer

1. Build the executable with PyInstaller.
2. Use Inno Setup or a similar tool to bundle the contents of `dist/` into an installer.
   A sample Inno Setup script (`magos.iss`) is included.
   Open it with the Inno Setup Compiler and click **Compile** to generate `MAGOS-Setup.exe`.

