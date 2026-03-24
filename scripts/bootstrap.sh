#!/usr/bin/env bash
set -euo pipefail

uv sync --group dev
uv run pallas-dev doctor
