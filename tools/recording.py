"""Saves driver recordings sent over the brain's terminal into src/recordings/<name>.inc,
so the next upload replays them without an SD card.

    python3 tools/recording.py            # runs `pros terminal`; controller or brain plugged in
    python3 tools/recording.py log.txt    # or reads a saved terminal log
"""

import os
import re
import subprocess
import sys
import warnings
from pathlib import Path

################################################################
# --- Setup -------------------------------------------------- #
################################################################

recordings_dir = Path(__file__).resolve().parent.parent / "src" / "recordings"
pros           = os.path.expanduser("~/.local/bin/pros")

start_line = re.compile(r"=== recording (\S+) (\d+) rows ===")
end_line   = re.compile(r"=== end (\S+) ===")

################################################################
# --- Capture ------------------------------------------------ #
################################################################

def save(name, rows, expected):
    # Header plus one line per sample; a short count means the radio dropped lines
    if len(rows) != expected + 1:
        warnings.warn(f"{name}: got {len(rows) - 1} of {expected} rows, not saved. Record again or move closer.")
        return

    path = recordings_dir / f"{name}.inc"
    path.write_text('R"(' + "\n".join(rows) + '\n)"\n')
    print(f"Saved {path.relative_to(Path.cwd()) if path.is_relative_to(Path.cwd()) else path}: "
          f"{expected} rows. Rebuild and upload to keep it.", flush=True)


def capture(lines):
    name, expected, rows = None, 0, []
    for raw in lines:
        line = raw.rstrip("\r\n")
        print(line, flush=True)

        if match := start_line.search(line):
            name, expected, rows = match[1], int(match[2]), []
        elif name and end_line.search(line):
            save(name, rows, expected)
            name = None
        elif name:
            rows.append(line)


def terminal_lines():
    # pros is Python; unbuffered so lines arrive as the brain sends them
    env  = {**os.environ, "PYTHONUNBUFFERED": "1"}
    proc = subprocess.Popen([pros, "terminal"], stdout=subprocess.PIPE, text=True, env=env)
    try:
        yield from proc.stdout
    finally:
        proc.terminate()

################################################################
# --- Main --------------------------------------------------- #
################################################################

if __name__ == "__main__":
    try:
        capture(open(sys.argv[1]) if len(sys.argv) > 1 else terminal_lines())
    except KeyboardInterrupt:
        pass
