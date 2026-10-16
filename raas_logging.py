from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

_shared_path = Path(__file__).resolve().parent.parent / "raas_logging.py"

if _shared_path.exists():
    spec = importlib.util.spec_from_file_location("_raas_logging_shared", _shared_path)
    if spec is None or spec.loader is None:
        raise ModuleNotFoundError(f"Unable to load shared RAAS logger from {_shared_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    for attr_name in dir(module):
        if attr_name.startswith("__") and attr_name.endswith("__"):
            continue
        globals()[attr_name] = getattr(module, attr_name)
else:
    raise ModuleNotFoundError(
        "Shared RAAS logger module not found. Expected it at "
        f"{_shared_path.parent / 'raas_logging.py'}"
    )
