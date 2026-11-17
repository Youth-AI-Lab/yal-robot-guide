#!/bin/bash
# Renders each diagram NN-<name>.d2 of this folder into NN-<name>.svg, with a Kroki server.
# Each source is preceded by _style.d2 (the shared style).
#
# Kroki on this computer (the public image, nothing leaves the computer):
#   docker run -d --rm --name kroki -p 127.0.0.1:8000:8000 yuzutech/kroki
#   bash images/render.sh                 # every diagram
#   bash images/render.sh 01-overview.d2  # one only
#   docker stop kroki
#
# KROKI_URL chooses another Kroki server.

set -euo pipefail
cd "$(dirname "$0")"

KROKI_URL="${KROKI_URL:-http://127.0.0.1:8000}"

if [ "$#" -gt 0 ]; then sources=("$@"); else sources=([0-9][0-9]-*.d2); fi

for src in "${sources[@]}"; do
  out="${src%.d2}.svg"
  echo -n "  $src -> $out ... "
  if cat _style.d2 "$src" | curl -sf -X POST "$KROKI_URL/d2/svg" \
       -H "Content-Type: text/plain" --data-binary @- -o "$out.tmp"; then
    mv "$out.tmp" "$out"
    echo "OK"
  else
    rm -f "$out.tmp"
    echo "ERROR (Kroki not reachable, or the source is wrong)"
    exit 1
  fi
done
