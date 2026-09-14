"""Package this task's report, raw data and source without environments/caches."""

import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "reports" / "task1_submission.zip")
    parser.add_argument("--replace", action="store_true", help="Explicitly replace this output archive")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists() and not args.replace:
        parser.error(f"Archive already exists; choose a new filename: {output}")
    files = []
    for folder in (ROOT / "reports" / "task1", ROOT / "outputs" / "task1" / "20260911_task1"):
        files.extend(path for path in folder.rglob("*") if path.is_file())
    files.extend(path for path in (ROOT / "vdm_lab").rglob("*") if path.is_file() and path.suffix in {".py", ".md"})
    files += [ROOT / relative for relative in (
        ".gitignore", "README.md", "ENVIRONMENT.md", "TEACHING_GUIDE.md", "requirements.txt", "requirements.lock.txt",
        "run_experiment.py", "scripts/run_task1.py", "scripts/analyze_task1.py", "scripts/package_task1.py", "tests/test_task1.py",
        "vdm_lab/KMLM.png", "vdm_lab/exp_cm.png", "VehicleDynamicsMobility_01_BicycleModel.pdf",
    )]
    files = sorted(set(files))
    for path in files:
        if not path.is_file():
            raise FileNotFoundError(path)
        if path == output or "__pycache__" in path.parts or ".venv" in path.parts:
            raise ValueError(f"Unexpected archive member: {path}")
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, "w" if args.replace else "x", compression=ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            archive.write(path, path.relative_to(ROOT))
        archive.writestr("BUNDLE_MANIFEST.json", json.dumps({"entry_report": "reports/task1/report.md", "sha256": hashes}, ensure_ascii=False, indent=2) + "\n")
    with ZipFile(output) as archive:
        assert archive.testzip() is None
        for relative, expected in hashes.items():
            assert hashlib.sha256(archive.read(relative)).hexdigest() == expected
    print(f"Verified archive: {output} ({output.stat().st_size / 1024**2:.2f} MiB, {len(files)} files + checksum manifest)")


if __name__ == "__main__":
    main()
