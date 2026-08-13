#!/usr/bin/env bash
# USB-audio loopback capture harness for macOS.
#
# Plays an input WAV to the Daisy's USB *output* and simultaneously records from
# its USB *input*, so whatever the device does between them (a pass-through
# loopback, or a DSP core once wired in) is captured for analysis. See README.md
# for the full story — including the two macOS gotchas this script exists to
# defeat:
#
#   1. Bluetooth outputs (AirPods) aggressively re-grab the default device, so we
#      set the Daisy as the default OUTPUT immediately before `afplay` and restore
#      it afterward.
#   2. The Daisy's avfoundation INPUT index changes every time it re-enumerates
#      (e.g. after a reflash). We auto-detect it by NAME every run rather than
#      hard-coding an index — the bug that made every early recording silent
#      (we were recording the MacBook mic).
#
# Usage:
#   ./loopback.sh <input.wav> <output.wav> [record_seconds]
#
# Requires: ffmpeg, switchaudio-osx  (brew install ffmpeg switchaudio-osx)
set -euo pipefail

IN_WAV="${1:?usage: loopback.sh <input.wav> <output.wav> [record_seconds]}"
OUT_WAV="${2:?usage: loopback.sh <input.wav> <output.wav> [record_seconds]}"
REC_SECS="${3:-6}"
DEV_MATCH="${DAISY_MATCH:-Daisy}"   # substring identifying the device (override for pod-midi etc.)

for tool in ffmpeg SwitchAudioSource afplay; do
  command -v "$tool" >/dev/null 2>&1 || {
    echo "error: '$tool' not found. Install: brew install ffmpeg switchaudio-osx" >&2; exit 1; }
done
[ -f "$IN_WAV" ] || { echo "error: input WAV '$IN_WAV' not found" >&2; exit 1; }

# --- auto-detect the Daisy, by name, on BOTH device lists ---------------------
# avfoundation input index (the number in "[N] Daisy ..."). `sed -n '1..p'`
# takes the first match without `head` (which would SIGPIPE the pipeline and,
# under `set -e -o pipefail`, kill the script); `|| true` guards a no-match.
IN_IDX="$(ffmpeg -f avfoundation -list_devices true -i "" 2>&1 \
  | awk '/AVFoundation audio devices/{a=1} a' \
  | grep -i "$DEV_MATCH" \
  | sed -nE '1s/.*\[([0-9]+)\].*/\1/p' || true)"
[ -n "${IN_IDX:-}" ] || { echo "error: no avfoundation input matching '$DEV_MATCH' — is the board plugged in and running the USB-audio app?" >&2; exit 1; }
# SwitchAudioSource output device name (exact string).
OUT_NAME="$(SwitchAudioSource -a -t output | grep -i "$DEV_MATCH" | sed -n '1p' || true)"
[ -n "${OUT_NAME:-}" ] || { echo "error: no CoreAudio output matching '$DEV_MATCH'" >&2; exit 1; }
echo "device: input avfoundation [$IN_IDX], output \"$OUT_NAME\""

# --- restore the previous default output on exit ------------------------------
PREV_OUT="$(SwitchAudioSource -c -t output)"
restore() { SwitchAudioSource -s "$PREV_OUT" -t output >/dev/null 2>&1 || true; }
trap restore EXIT
echo "saved default output: \"$PREV_OUT\" (will restore)"

# --- record (by explicit index, robust) while playing (via default) -----------
echo "recording ${REC_SECS}s from the device input while playing '$IN_WAV' to it..."
ffmpeg -y -f avfoundation -i ":${IN_IDX}" -t "$REC_SECS" -ac 2 -ar 48000 \
  -c:a pcm_s16le "$OUT_WAV" >/dev/null 2>&1 &
REC_PID=$!
# let the capture stream come up (device IN alt-setting active) before playing
python3 -c 'import time; time.sleep(1.5)'
# set default + play back-to-back so a Bluetooth device can't steal it in between
SwitchAudioSource -s "$OUT_NAME" -t output >/dev/null
afplay "$IN_WAV"
wait "$REC_PID"
echo "captured -> $OUT_WAV"
