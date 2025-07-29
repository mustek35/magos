# MAGOS Radar Application

This project contains a PyQt-based GUI for interacting with the MAGOS radar.
Authentication to the web interface is performed via a token obtained from the
radar's API, allowing the embedded browser to open without asking for
credentials.

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

## Debugging Web Access

To troubleshoot issues accessing the embedded web interface, enable debug
messages by setting the environment variable `MAGOS_DEBUG=1` or passing the
`--debug` option when launching the application:

```bash
MAGOS_DEBUG=1 python magos.py
# or
python magos.py --debug
```

With debug mode enabled the console and the log panel will show the
authentication response and the exact URL loaded in the web view.

