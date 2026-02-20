#!/usr/bin/env python3
"""
Check if rosbag has metadata.yaml and reindex if missing.
"""

import argparse
import sys
from pathlib import Path
from rosbag2_py import Reindexer, StorageOptions


def reindex_bag_if_needed(bag_path: Path, storage_id: str = "sqlite3") -> bool:
    """
    Check if bag has metadata.yaml and reindex if missing.

    Args:
        bag_path: Path to bag directory or file
        storage_id: Storage format (sqlite3 or mcap)

    Returns:
        True if reindexing was performed, False if not needed
    """
    bag_path = Path(bag_path)

    # Determine if it's a directory or file
    if bag_path.is_dir():
        bag_dir = bag_path
        # Check storage format from files
        if list(bag_dir.glob("*.mcap")):
            storage_id = "mcap"
        elif list(bag_dir.glob("*.db3")):
            storage_id = "sqlite3"
    else:
        # It's a file
        bag_dir = bag_path.parent
        if bag_path.suffix == ".mcap":
            storage_id = "mcap"
        elif bag_path.suffix == ".db3":
            storage_id = "sqlite3"

    # Check if metadata.yaml exists
    metadata_file = bag_dir / "metadata.yaml"
    if metadata_file.exists():
        print(f"✓ Metadata exists: {bag_dir}")
        return False

    # Need to reindex
    print(f"⚠ Missing metadata.yaml, reindexing: {bag_dir}")

    try:
        storage_options = StorageOptions(uri=str(bag_dir), storage_id=storage_id)
        reindexer = Reindexer()
        reindexer.reindex(storage_options)
        print(f"✓ Reindexed successfully")
        return True
    except Exception as e:
        print(f"✗ Failed to reindex: {e}", file=sys.stderr)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Reindex rosbag if metadata.yaml is missing"
    )
    parser.add_argument("bag_path", type=Path, help="Path to bag directory or file")
    parser.add_argument(
        "--storage-id",
        default="sqlite3",
        choices=["sqlite3", "mcap"],
        help="Storage format (default: sqlite3, auto-detected from files)",
    )

    args = parser.parse_args()

    if not args.bag_path.exists():
        print(f"ERROR: Bag path not found: {args.bag_path}", file=sys.stderr)
        sys.exit(1)

    reindexed = reindex_bag_if_needed(args.bag_path, args.storage_id)
    sys.exit(0 if reindexed or Path(args.bag_path).is_dir() else 1)


if __name__ == "__main__":
    main()
