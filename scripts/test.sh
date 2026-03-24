#!/usr/bin/env bash
set -euo pipefail

uv run pallas-dev test "$@"
