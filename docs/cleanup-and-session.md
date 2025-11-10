SPIFFS Cleanup and Session Notes
=================================

This document describes the one-shot SPIFFS cleanup/format helpers added to the
firmware and summarizes the interactive session that used them.

Why these helpers exist
-----------------------
The project moved from a heavy per-field runtime UI (many individual
`ConfigItem`s persisted in SPIFFS) to a compact JSON-overrides system. Old
persisted per-field entries on SPIFFS can still appear in the web UI and cause
clutter. The helpers provide a safe, temporary way to remove legacy saved
entries.

How to use (safe workflow)
--------------------------
1. Backup current runtime config
   - Open the web UI and copy `/System/ConfigExport` to a local file.

2. Enable cleanup temporarily
   - Edit `platformio.ini` and add one of the flags under `[env:halmet]`:
     - `-D FORCE_SPIFFS_CLEANUP=1` — file-by-file cleanup (keeps `/www`).
     - `-D FORCE_SPIFFS_FORMAT=1` — full `SPIFFS.format()` (destructive).

3. Build and flash
   - Build and upload the firmware. Monitor the serial output. The firmware
     prints actions taken and writes a marker file (`/persist_cleanup_done` or
     `/persist_format_done`) so the action does not repeat.

4. Verify and revert
   - Open the web UI and confirm old entries are gone.
   - Remove the `-D FORCE_*` flag(s) from `platformio.ini` and rebuild, or
     revert the commit.

Implementation Details
----------------------
The SPIFFS cleanup/format functionality is implemented as compile-time guarded
code blocks in `main.cpp`. The code is inactive by default and only activates
when specific preprocessor flags are defined.

### Available Flags

- `FORCE_SPIFFS_CLEANUP`
  - Performs file-by-file cleanup at boot
  - Iterates over SPIFFS and removes all files except the `/www` folder
  - Preserves web UI static assets
  - Writes `/persist_cleanup_done` marker to prevent re-execution

- `FORCE_SPIFFS_FORMAT`
  - Calls `SPIFFS.format()` to erase the entire SPIFFS filesystem
  - Most destructive option - removes ALL files including `/www`
  - Writes `/persist_format_done` marker to prevent re-execution

### Code Structure

The implementation is located in `main.cpp` after the SensESP app initialization.
It's wrapped in `#if defined(FORCE_SPIFFS_CLEANUP)` and `#if defined(FORCE_SPIFFS_FORMAT)`
blocks to ensure it's inactive by default.

### Safety Features

- **Guarded by preprocessor flags**: Code only compiles when flags are explicitly set
- **One-shot execution**: Marker files prevent re-execution on subsequent boots
- **Serial logging**: Prints all actions taken for verification
- **Repository safety**: No FORCE_SPIFFS_* flags are enabled in the default `platformio.ini`

### Usage in platformio.ini

To temporarily enable cleanup, add build flags under `[env:halmet]`:

```
[env:halmet]
build_flags =
    ${pioarduino.build_flags}
    ${esp32.build_flags}
    -D ARDUINOJSON_ENABLE_COMMENTS=1
    -D FORCE_SPIFFS_CLEANUP=1
```

### Important Notes

- The repository currently does NOT enable any FORCE_SPIFFS_* flags in `platformio.ini`
- File-by-file cleanup preserves `/www` to keep web UI assets
- Format option erases everything and should only be used when planning to reflash UI assets
- Always backup configuration before using these features