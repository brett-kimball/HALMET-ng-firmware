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

Session summary (Nov 10, 2025)
------------------------------
- The assistant added the guarded helpers to `src/main.cpp` and briefly
  enabled the flags to run a one-shot `SPIFFS.format()` followed by a
  file-by-file cleanup on the device. The format succeeded. Some watchdog
  messages were emitted during the heavy I/O activity but the device booted.
- After verification, the assistant removed the FORCE_* flags from
  `platformio.ini` so future builds are safe by default.

Where the chat log is stored
---------------------------
This repo now contains `docs/cleanup-and-session.md` summarizing the steps and
outcome. It does not include the full chat transcript. If you want a full
chat export, copy the conversation text manually into `docs/session-full.txt`.

Repository branches
-------------------
- `json-clean-start` — experimental branch containing the JSON helpers and
  cleanup code (recommended to push to remote for backup).
- `main-experimental` — another experimental branch present locally.

Safety notes
------------
- Do NOT leave FORCE_* flags enabled in `platformio.ini` in your normal
  development or CI builds.
- Always backup `ConfigExport` and any important settings before running
  destructive actions.