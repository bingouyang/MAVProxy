"""
update_ponds_json.py
---------------------
Merges all *_features.json files in a directory into ponds.json,
skipping features whose pond number already exists.

Usage:
    python update_ponds_json.py                            # uses current directory
    python update_ponds_json.py --dir /path/to/jsons       # specify directory
    python update_ponds_json.py --out /path/to/ponds.json  # specify output file

The script looks for any file matching *_features.json (e.g. farm_features.json,
basler_features.json, future_farm_features.json) and merges all features into
ponds.json, preserving existing entries and appending new ones.
"""

import json
import argparse
import glob
import os


def load_json(path):
    with open(path, 'r') as f:
        return json.load(f)


def save_json(data, path):
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)


def merge_into_ponds(ponds_path, feature_files):
    # Load existing ponds.json
    if os.path.exists(ponds_path):
        ponds = load_json(ponds_path)
        print(f"Loaded {len(ponds['features'])} existing features from {ponds_path}")
    else:
        ponds = {"type": "FeatureCollection", "features": []}
        print(f"No existing {ponds_path} found — creating new file")

    # Build set of existing pond numbers to detect duplicates
    existing = {str(f['properties']['number']) for f in ponds['features']}

    added = 0
    skipped = 0

    for fpath in sorted(feature_files):
        fname = os.path.basename(fpath)
        try:
            data = load_json(fpath)
        except Exception as e:
            print(f"  ERROR reading {fname}: {e}")
            continue

        features = data.get('features', [])
        print(f"\nProcessing {fname} ({len(features)} features):")

        for feature in features:
            number = str(feature['properties'].get('number', ''))
            if not number:
                print(f"  SKIP — feature has no 'number' property")
                skipped += 1
                continue

            if number in existing:
                print(f"  SKIP pond_{number} — already in ponds.json")
                skipped += 1
            else:
                ponds['features'].append(feature)
                existing.add(number)
                print(f"  ADD  pond_{number}")
                added += 1

    print(f"\nDone. Added {added} new feature(s), skipped {skipped} duplicate(s).")
    return ponds


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Merge *_features.json files into ponds.json")
    parser.add_argument('--dir', default='.',
                        help="Directory containing *_features.json files (default: current dir)")
    parser.add_argument('--out', default=None,
                        help="Output ponds.json path (default: <dir>/ponds.json)")
    args = parser.parse_args()

    search_dir = os.path.abspath(args.dir)
    ponds_path = args.out or os.path.join(search_dir, 'ponds.json')

    # Find all *_features.json files, excluding ponds.json itself
    pattern = os.path.join(search_dir, '*_features.json')
    feature_files = [f for f in glob.glob(pattern)
                     if os.path.abspath(f) != os.path.abspath(ponds_path)]

    if not feature_files:
        print(f"No *_features.json files found in {search_dir}")
        exit(1)

    print(f"Found {len(feature_files)} feature file(s): "
          f"{[os.path.basename(f) for f in feature_files]}\n")

    merged = merge_into_ponds(ponds_path, feature_files)
    save_json(merged, ponds_path)
    print(f"\nSaved {len(merged['features'])} total features to {ponds_path}")
