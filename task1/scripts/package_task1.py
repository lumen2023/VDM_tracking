"""Package the single task1 folder; reuse the host repository's simulation code."""

import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

TASK_ROOT = Path(__file__).resolve().parents[1]
ROOT = TASK_ROOT.parent
LOCAL_ONLY = {"artifacts", "outputs", "environment_checks", ".venv", "__pycache__"}


def package_files():
    files = []
    for path in TASK_ROOT.rglob("*"):
        relative = path.relative_to(TASK_ROOT)
        if not path.is_file() or any(part in LOCAL_ONLY for part in relative.parts):
            continue
        if path.suffix in {".pyc", ".pyo", ".zip"} or relative.as_posix() == "BUNDLE_MANIFEST.json":
            continue
        files.append(path)
    for relative in ("README.md", "ENVIRONMENT.md", "requirements.lock.txt", "reports/report.md",
                     "data/20260911_task1/manifest.json", "data/20260911_task1/results.json"):
        if TASK_ROOT / relative not in files:
            raise FileNotFoundError(TASK_ROOT / relative)
    return sorted(files)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=TASK_ROOT / "artifacts" / "task1_submission.zip")
    parser.add_argument("--replace", action="store_true", help="Explicitly replace this output archive")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.suffix.lower() != ".zip":
        parser.error("Output must be a .zip archive")
    if output.exists() and not args.replace:
        parser.error(f"Archive already exists; choose a new filename: {output}")
    files = package_files()
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, "w" if args.replace else "x", compression=ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            archive.write(path, path.relative_to(ROOT))
        archive.writestr("task1/BUNDLE_MANIFEST.json", json.dumps({
            "entry_report": "task1/reports/report.md",
            "requires": "Place task1/ inside the VDM_tracking repository; the original vdm_lab/ code is not duplicated.",
            "sha256": hashes,
        }, ensure_ascii=False, indent=2) + "\n")
    with ZipFile(output) as archive:
        assert archive.testzip() is None
        for relative, expected in hashes.items():
            assert hashlib.sha256(archive.read(relative)).hexdigest() == expected
    print(f"Verified archive: {output} ({output.stat().st_size / 1024**2:.2f} MiB, {len(files)} files + checksum manifest)")


if __name__ == "__main__":
    main()
