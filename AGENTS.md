# AGENTS.md

SpiriMav runs on a drone's companion computer: it bridges the flight controller's MAVLink to a SpiriSynq `Robot` object. Python >=3.13, managed with `uv` (`uv.lock` is committed). Generated from the SpiriProjectTemplates copier template.

## Commands

- Tests: `uv run pytest`.
- Docs: `make docs` (Sphinx, myst markdown).

## Safety design — read `docs/design.md` before touching heartbeats or failsafes

- The companion is a **secondary** GCS. It must never use the FCU's GCS sysid (`SYSID_MYGCS` / `MAV_GCS_SYSID`); that belongs to QGC, so the FCU's native GCS failsafe watches QGC directly. A companion heartbeat on that sysid would mask a lost QGC.
- We don't own or see QGC's link. Never gate safety behaviour on whether we see QGC traffic; `gcs_seen` is display-only.
- `Robot.failsafe` mirrors what the FCU reports; SpiriMav doesn't invent failsafes.
- Commands come in as SpiriSynq `@remote_method` RPCs, not field writes from mirrors.

## Release conventions

- Every change gets a categorized entry in `CHANGES.md` (Features / Fixes / Improvements / Tests / Internal) under `## Unreleased`; commits use conventional-commit style (`feat:`, `fix:`, `refactor:`).
