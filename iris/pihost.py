"""Find the Pi's address for PC tools.

Order: --host argument, IRIS_PI_HOST env var, then HostName of `Host iris-pi`
in ~/.ssh/config. So when the Pi's IP changes you edit ~/.ssh/config only.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

SSH_ALIAS = "iris-pi"


def from_ssh_config(alias: str = SSH_ALIAS, path: Optional[Path] = None) -> Optional[str]:
    path = path or Path.home() / ".ssh" / "config"
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return None
    in_block = False
    for raw in lines:
        parts = raw.strip().split(None, 1)
        if len(parts) != 2 or parts[0].startswith("#"):
            continue
        key, val = parts[0].lower(), parts[1].strip()
        if key == "host":
            in_block = alias in val.split()
        elif in_block and key == "hostname":
            return val
    return None


def resolve(host: Optional[str] = None) -> str:
    h = host or os.environ.get("IRIS_PI_HOST") or from_ssh_config()
    if not h:
        raise SystemExit("Pi address unknown: pass --host, set IRIS_PI_HOST, "
                         "or add 'Host iris-pi' with HostName to ~/.ssh/config")
    return h
