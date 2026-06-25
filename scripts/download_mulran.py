#!/usr/bin/env python3
"""
MulRan dataset indirici — AutoNav-SLAM test verisi.
Dataset: https://sites.google.com/view/mulran-pr/dataset

Kullanım:
    python scripts/download_mulran.py --sequence KAIST01 --out ./data
"""
import argparse
import subprocess
import sys
from pathlib import Path

SEQUENCES = {
    "KAIST01": "https://urserver.kaist.ac.kr/publicData/MulRan/KAIST01/",
    "KAIST02": "https://urserver.kaist.ac.kr/publicData/MulRan/KAIST02/",
    "Riverside01": "https://urserver.kaist.ac.kr/publicData/MulRan/Riverside01/",
    "DCC01": "https://urserver.kaist.ac.kr/publicData/MulRan/DCC01/",
}

# Küçük test paketi (sadece ilk 100 scan — ~500MB)
SMALL_TEST_FILES = [
    "Ouster/",       # LiDAR pointcloud'lar
    "sensor_data/",  # IMU + GPS
    "global_pose.csv",
]


def main() -> None:
    parser = argparse.ArgumentParser(description="MulRan dataset indirici")
    parser.add_argument("--sequence", default="KAIST01", choices=list(SEQUENCES.keys()))
    parser.add_argument("--out", default="./data/mulran", type=Path)
    parser.add_argument("--full", action="store_true", help="Tam dataset indir (>10GB)")
    args = parser.parse_args()

    out_dir = args.out / args.sequence
    out_dir.mkdir(parents=True, exist_ok=True)

    base_url = SEQUENCES[args.sequence]
    print(f"Sequence: {args.sequence}")
    print(f"Hedef: {out_dir}")
    print(f"Mod: {'Tam' if args.full else 'Test (~500MB ilk 100 scan)'}\n")

    files = ["."] if args.full else SMALL_TEST_FILES

    for f in files:
        url = base_url + f
        cmd = [
            "wget", "-r", "-np", "-nH", "--cut-dirs=4",
            "-P", str(out_dir), url,
        ]
        if not args.full:
            # Sadece ilk 100 dosya
            cmd += ["--quota=600m"]

        print(f"İndiriliyor: {url}")
        try:
            subprocess.run(cmd, check=True)
        except FileNotFoundError:
            print("wget bulunamadı. curl ile deniyor...")
            subprocess.run(["curl", "-L", "-o", str(out_dir / Path(f).name), url], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Hata: {e}")
            sys.exit(1)

    print(f"\nTamamlandi! Veriler: {out_dir}")
    print("\nROS2 ile rosbag oynatmak icin:")
    print(f"  ros2 bag play {out_dir}/rosbag/")
    print("  veya MulRan'i rosbag'e donusturmek icin:")
    print("  pip install mulran2bag && mulran2bag --input", out_dir)


if __name__ == "__main__":
    main()
