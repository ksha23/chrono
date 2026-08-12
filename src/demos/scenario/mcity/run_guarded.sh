#!/bin/bash
# Run a demo with an RSS ceiling, so an experiment that turns out to be too big kills itself
# instead of the desktop. Usage: run_guarded.sh <limit_gb> <command...>
set -u
LIMIT_GB="$1"; shift
"$@" &
PID=$!
PEAK=0
while kill -0 "$PID" 2>/dev/null; do
  RSS=$(ps -o rss= -p "$PID" 2>/dev/null | tr -d ' ')
  [ -z "$RSS" ] && break
  [ "$RSS" -gt "$PEAK" ] && PEAK=$RSS
  if [ "$RSS" -gt $((LIMIT_GB * 1024 * 1024)) ]; then
    echo "  !! RSS $((RSS/1048576)) GB over ${LIMIT_GB} GB -- killing" >&2
    kill -9 "$PID" 2>/dev/null; wait "$PID" 2>/dev/null
    echo "  peak before kill: $((PEAK/1048576)) GB" >&2
    exit 2
  fi
  sleep 0.2
done
wait "$PID" 2>/dev/null
RC=$?
echo "  peak RSS: $(echo "$PEAK" | awk '{printf "%.2f", $1/1048576}') GB" >&2
exit $RC
