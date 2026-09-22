#!/usr/bin/env bash
# Live visualization must start at the latest buffered sample after reconnecting.
exec rerun --newest-first "$@"
