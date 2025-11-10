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

Files changed
-------------
- `src/main.cpp` — added guarded code for `FORCE_SPIFFS_CLEANUP` and
  `FORCE_SPIFFS_FORMAT` plus a detailed instruction block.
- `platformio.ini` — flags may be toggled locally to enable the actions for
  a single build.

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