from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent
ZIP_PATH = ROOT.parent / f"{ROOT.name}_debug.zip"

EXCLUDED_DIRS = {
    ".git",
    ".idea",
    ".vscode",
    ".venv",
    "venv",
    "env",
    "__pycache__",
    ".pytest_cache",
    ".mypy_cache",
    ".ruff_cache",
    "node_modules",
    "site-packages",
    "dist",
    "build",
    "htmlcov",
    ".tox",
}

EXCLUDED_FILES = {
    ".DS_Store",
    "Thumbs.db",
    "desktop.ini",
}

EXCLUDED_SUFFIXES = {
    ".pyc",
    ".pyo",
    ".log",
    ".tmp",
    ".swp",
    ".bak",
    ".engine",
    ".onnx",
    ".pt",
    ".pth",
    ".weights",
}

EXCLUDED_PATH_PARTS = {
    "runs",
    "wandb",
    "mlruns",
}

def should_exclude(path: Path) -> bool:
    relative = path.relative_to(ROOT)
    parts = set(relative.parts)

    if parts & EXCLUDED_DIRS:
        return True

    if parts & EXCLUDED_PATH_PARTS:
        return True

    if path.name in EXCLUDED_FILES:
        return True

    if path.suffix.lower() in EXCLUDED_SUFFIXES:
        return True

    if path.resolve() == ZIP_PATH.resolve():
        return True

    return False

def create_zip():
    count = 0

    with zipfile.ZipFile(
        ZIP_PATH, "w", compression=zipfile.ZIP_DEFLATED
    ) as archive:
        for path in ROOT.rglob("*"):
            if not path.is_file() or should_exclude(path):
                continue

            archive.write(path, path.relative_to(ROOT))
            count += 1

    print(f"ZIP created: {ZIP_PATH}")
    print(f"Files included: {count}")


if __name__ == "__main__":
    create_zip()