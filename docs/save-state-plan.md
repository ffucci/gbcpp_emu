# Save-State Implementation Plan

This note records the planned save-state feature and the ownership split for implementation.

## Summary

Save states should be separate from the existing `.battery` cartridge RAM saves. A save state captures the current emulator execution point so the same ROM can be restored later without relying on in-game save support.

The target format is a binary `.state` file with:

- A small header containing a magic value, save-state format version, ROM identity metadata, and payload size.
- ROM identity validation using stable cartridge metadata such as title, checksum, size, and cartridge type.
- A payload containing explicit snapshots of emulator components rather than raw object memory.

## State Coverage

The save-state payload should include:

- CPU registers, flags, fetched/current instruction bookkeeping where needed, interrupt state, halt/running state, IME state, cycle/tick counters, and timer context.
- MMU-owned state, including WRAM, HRAM, interrupt enable register, DMA context, LCD/device state, and PPU state.
- PPU/LCD state needed for exact mid-frame restore, including VRAM, OAM, FIFO/fetcher state, line timing, window line, video buffer, LCD registers, palettes, and mode fields.
- Device/input state, including serial registers and gamepad selection/pressed bits.
- Cartridge state, including external RAM contents, selected ROM/RAM banks, RAM enable state, dirty/battery-save flags, and mapper-specific fields.
- MBC-specific details for the currently supported cartridge types: `NoMBC`, `MBC1`, `MBC3`, and `MBC5`.

## Thread Safety

Save and restore operations must run through `EmulatorSession`, not through UI code directly. The CPU runs on a worker thread, so snapshot/restore needs a clear synchronization point.

The intended approach is to add state accessors protected by a mutex around one complete CPU loop iteration and around snapshot/restore operations. Restoring should only happen when a ROM is already loaded, and the state file must be rejected if its ROM identity does not match the current ROM.

## Ownership Split

The user will implement:

- Core snapshot/restore APIs across CPU, MMU, cartridge, timer, PPU/LCD, DMA, RAM, and device state.
- Binary serialization/deserialization and file validation.

Codex will implement later:

- `EmulatorSession::save_state(path)` and `EmulatorSession::load_state(path)` wiring.
- SDL UI shortcuts and dialogs for save and restore.
- Focused GoogleTest coverage for round trips, restore behavior, mismatch handling, and session/UI-facing flows.
- README/docs updates after the feature lands.

## Test Plan

Planned coverage:

- Serializer round-trip tests for primitive fields, vectors, fixed arrays, invalid magic, invalid version, and ROM mismatch.
- Component snapshot tests that mutate state, restore a previous snapshot, and assert exact values.
- Cartridge snapshot tests for `NoMBC`, `MBC1`, `MBC3`, and `MBC5`.
- Integration smoke test: load a test ROM, run briefly, save, advance execution, restore, and verify the restored state is deterministic.
- Manual SDL check once UI wiring exists: save with `Ctrl+S`, advance gameplay, restore with `Ctrl+R`, and confirm execution resumes from the saved point.

## Assumptions

- Save states only need compatibility within this emulator version.
- Binary format is preferred because emulator state includes large exact byte buffers.
- Exact mid-frame restore is desired, so PPU FIFO/video buffer and LCD timing fields are included.
- Existing `.battery` behavior remains unchanged.
