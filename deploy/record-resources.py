#!/usr/bin/env python3
"""Low-rate host resource evidence, owned and stopped by the recording launch."""

import argparse
import json
from pathlib import Path
import shutil
import signal
import time


def cpu_ticks():
    values = [int(v) for v in Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
    return sum(values), sum(values[3:5])


def memory():
    rows = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value = line.split(':', 1)
        if key in {'MemTotal', 'MemAvailable'}:
            rows[key] = int(value.split()[0]) * 1024
    return rows


def thermal():
    zones = {}
    for path in Path('/sys/class/thermal').glob('thermal_zone*'):
        try:
            zones[path.name] = {'type': (path / 'type').read_text().strip(),
                                'temperature_c': int((path / 'temp').read_text()) / 1000}
        except (OSError, ValueError):
            continue
    return zones


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    previous = cpu_ticks()
    with (args.session / 'resources.jsonl').open('x', buffering=1) as out:
        while running:
            total, idle = cpu_ticks()
            delta = total - previous[0]
            row = {'utc_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
                   'host_cpu_busy_percent': 100 * (1 - (idle - previous[1]) / delta) if delta else None,
                   'host_memory_bytes': memory(), 'thermal_zones': thermal(),
                   'free_bytes': shutil.disk_usage(args.session).free,
                   'bag_bytes': sum(p.stat().st_size for p in (args.session / 'bag').glob('*.mcap'))}
            out.write(json.dumps(row, allow_nan=False) + '\n')
            previous = total, idle
            for _ in range(50):
                if not running:
                    break
                time.sleep(.1)


if __name__ == '__main__':
    main()
