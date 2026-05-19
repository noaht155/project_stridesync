"""
StrideSync WiFi log fetcher.

Downloads binary log files from the ESP32 SD card over HTTP (Mode 11),
then converts them to CSV ready for the dashboard.

Usage:
    python fetch_log.py --ip 192.168.1.42              # list files, download latest
    python fetch_log.py --ip 192.168.1.42 --list       # list files only
    python fetch_log.py --ip 192.168.1.42 --file run_003.bin
    python fetch_log.py --ip 192.168.1.42 --delete run_001.bin
    python fetch_log.py --ip 192.168.1.42 --out runs/  # save to specific folder

The ESP32 must be in Mode 11 (WiFi file server) and on the same network.
Its IP address is printed to the Serial monitor on boot.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import requests

# data_loader lives one directory up from this file
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from data_loader import load_binary

TIMEOUT_CONNECT = 5   # seconds
TIMEOUT_DOWNLOAD = 120
CHUNK_SIZE = 8192


def _base_url(ip: str) -> str:
    return f'http://{ip}'


def list_files(ip: str) -> list[str]:
    """Return list of .bin filenames on the SD card."""
    resp = requests.get(f'{_base_url(ip)}/files', timeout=TIMEOUT_CONNECT)
    resp.raise_for_status()
    return resp.json().get('files', [])


def download_file(ip: str, filename: str, out_dir: Path) -> Path:
    """Stream a file from the ESP32 to out_dir. Returns local path."""
    url = f'{_base_url(ip)}/download?name={filename}'
    dest = out_dir / filename

    print(f'Downloading {filename} from {ip}...')
    with requests.get(url, stream=True, timeout=TIMEOUT_DOWNLOAD) as resp:
        resp.raise_for_status()
        total = int(resp.headers.get('Content-Length', 0))
        received = 0
        with open(dest, 'wb') as f:
            for chunk in resp.iter_content(CHUNK_SIZE):
                f.write(chunk)
                received += len(chunk)
                if total:
                    bar_len = 30
                    filled = int(bar_len * received / total)
                    bar = '#' * filled + '-' * (bar_len - filled)
                    pct = received / total * 100
                    print(f'\r  [{bar}] {pct:5.1f}%  {received}/{total} B',
                          end='', flush=True)
    print(f'\n  Saved: {dest}')
    return dest


def delete_file(ip: str, filename: str) -> None:
    url = f'{_base_url(ip)}/delete?name={filename}'
    resp = requests.get(url, timeout=TIMEOUT_CONNECT)
    resp.raise_for_status()
    print(f'Deleted {filename} from SD card.')


def convert_to_csv(bin_path: Path, keep_bin: bool = False) -> Path:
    """Convert binary log to CSV. Returns CSV path."""
    print(f'Converting {bin_path.name} to CSV...')
    df = load_binary(bin_path)
    csv_path = bin_path.with_suffix('.csv')
    df.to_csv(csv_path, index=False)
    rows = len(df)
    channels = df['channel'].nunique() if 'channel' in df.columns else '?'
    timestamps = df['timestamp_us'].nunique() if 'timestamp_us' in df.columns else '?'
    duration_s = (df['timestamp_us'].max() - df['timestamp_us'].min()) / 1e6 \
        if 'timestamp_us' in df.columns and len(df) > 1 else 0

    print(f'  {rows:,} records | {channels} channels | '
          f'{timestamps:,} timestamps | {duration_s:.1f}s run duration')
    print(f'  Saved: {csv_path}')

    if not keep_bin:
        bin_path.unlink()
        print(f'  Removed binary: {bin_path.name}')

    return csv_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description='StrideSync WiFi log fetcher — ESP32 must be in Mode 11'
    )
    parser.add_argument('--ip',     required=True, help='ESP32 IP address (see Serial monitor)')
    parser.add_argument('--file',   help='Specific filename to download (default: latest .bin)')
    parser.add_argument('--list',   action='store_true', help='List files only, do not download')
    parser.add_argument('--delete', metavar='FILENAME', help='Delete a file from the SD card')
    parser.add_argument('--out',    default='.', help='Output directory for downloaded files')
    parser.add_argument('--keep-bin', action='store_true',
                        help='Keep the binary file after CSV conversion')
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Delete mode
    if args.delete:
        try:
            delete_file(args.ip, args.delete)
        except requests.RequestException as e:
            print(f'Error: {e}')
            sys.exit(1)
        return

    # List files on SD
    try:
        print(f'Connecting to ESP32 at {args.ip}...')
        files = list_files(args.ip)
    except requests.ConnectionError:
        print(f'Could not reach {args.ip} — is the ESP32 in Mode 11 and on the same network?')
        sys.exit(1)
    except requests.RequestException as e:
        print(f'Error listing files: {e}')
        sys.exit(1)

    bin_files = [f for f in files if f.endswith('.bin')]

    if not bin_files:
        print('No .bin log files found on SD card.')
        print('Run an outdoor session in Mode 5 first.')
        return

    print(f'Found {len(bin_files)} log file(s) on SD card:')
    for f in bin_files:
        print(f'  {f}')

    if args.list:
        return

    # Pick target file
    target = args.file if args.file else sorted(bin_files)[-1]
    if target not in bin_files:
        print(f'File not found on SD card: {target}')
        sys.exit(1)

    # Download
    try:
        bin_path = download_file(args.ip, target, out_dir)
    except requests.RequestException as e:
        print(f'Download error: {e}')
        sys.exit(1)

    # Convert
    csv_path = convert_to_csv(bin_path, keep_bin=args.keep_bin)

    print(f'\nDone. Load this file in the StrideSync dashboard:')
    print(f'  {csv_path.resolve()}')


if __name__ == '__main__':
    main()
