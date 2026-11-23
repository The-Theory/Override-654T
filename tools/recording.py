"""Keeps the brain's last driver recording: run this, press "Send rec" on the brain's Tune page,
and it saves src/recordings/<name>.inc and quits. The next upload then replays it after restarts.

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
        warnings.warn(f"{name}: got {len(rows) - 1} of {expected} rows, not saved. Press Send rec again, or plug in by USB.")
        return False

    path = recordings_dir / f"{name}.inc"
    path.write_text('R"(' + "\n".join(rows) + '\n)"\n')
    print(f"Saved {path.relative_to(Path.cwd()) if path.is_relative_to(Path.cwd()) else path}: "
          f"{expected} rows. Upload to keep it.", flush=True)
    return True


def capture(lines):
    name, expected, rows = None, 0, []
    for raw in lines:
        line = raw.rstrip("\r\n")
        print(line, flush=True)

        if match := start_line.search(line):
            name, expected, rows = match[1], int(match[2]), []
        elif name and end_line.search(line):
            if save(name, rows, expected):
                return
            name = None
        elif name:
            rows.append(line)


def terminal_lines():
    # pros is Python; unbuffered so lines arrive as the brain sends them
    env  = {**os.environ, "PYTHONUNBUFFERED": "1"}
    proc = subprocess.Popen([pros, "terminal"], stdout=subprocess.PIPE, text=True, env=env)
    print('Listening. Press "Send rec" on the brain\'s Tune page.', flush=True)
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
