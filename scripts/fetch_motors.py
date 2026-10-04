"""Download the bundled motor thrust curves from the thrustcurve.org public API.

    python scripts/fetch_motors.py            # writes data/motors/*.eng (skips existing files)
    python scripts/fetch_motors.py --force    # re-download and overwrite (never Estes_C6.eng)

Each file gets a provenance header: thrustcurve.org motorId and simfileId,
the data source ("cert" = certification test data, preferred; "mfr" =
manufacturer data), the download date, and the published summary (total
impulse, burn time, masses) that tests/test_fleet_a1_motors.py checks the
curve against. The data points are reproduced unchanged.

Estes_C6.eng is never touched: the default rocket's regression pin depends
on that exact file.
"""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import json
import os
import sys
import tempfile
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MOTOR_DIR = REPO_ROOT / "data" / "motors"
API = "https://www.thrustcurve.org/api/v1/"

# (file stem, manufacturer as listed, common name, motorId). A-G hobby classes,
# plus the H-K high-power motors used by the fleet presets. Every motor here
# has at least one published curve integrating to within 1 % of its listed
# total impulse. Several popular motors do NOT (e.g. the AeroTech K1100T's
# certification curve integrates to +7.7 %, the Estes A8's to -7 %/-14 %);
# those were rejected, never rescaled.
MOTORS = [
    ("Estes_A10", "Estes", "A10", "5f4294d2000231000000000c"),
    ("Estes_B6", "Estes", "B6", "5f4294d20002310000000010"),
    ("Estes_D12", "Estes", "D12", "5f4294d20002310000000020"),
    ("Estes_E16", "Estes", "E16", "5f4294d200023100000003f8"),
    ("Estes_F15", "Estes", "F15", "5f4294d200023100000003f2"),
    ("AeroTech_G80T", "AeroTech", "G80", "5f4294d20002310000000068"),
    ("Cesaroni_H151", "Cesaroni", "H151", "5f4294d200023100000002de"),
    ("Cesaroni_I180", "Cesaroni", "I180", "5f4294d20002310000000284"),  # 338 Ns (the I218R, 319.6 Ns, is H by impulse)
    ("Cesaroni_J381", "Cesaroni", "J381", "5f4294d20002310000000287"),
    ("Cesaroni_K454", "Cesaroni", "K454", "5f4294d200023100000002a9"),
    ("Cesaroni_K940", "Cesaroni", "K940", "5f4294d200023100000003a9"),
]
SOURCE_PREFERENCE = ("cert", "mfr", "user")
IMPULSE_TOLERANCE = 0.01


def _impulse_error(data: str, published: float):
    """Relative error of a curve's integral vs the published impulse (None if unparseable)."""
    sys.path.insert(0, str(REPO_ROOT))
    from sim.motor import load_eng
    tmp = Path(tempfile.gettempdir()) / f"fetch_motors_{os.getpid()}.eng"
    try:
        tmp.write_text(data, encoding="utf-8")
        return (load_eng(tmp).total_impulse - published) / published
    except ValueError:
        return None
    finally:
        tmp.unlink(missing_ok=True)


def _g(v):
    """Strip float noise from the listing (127.90000000000002 -> 127.9)."""
    return "n/a" if v is None else f"{float(v):.6g}"


def post(path: str, body: dict) -> dict:
    req = urllib.request.Request(API + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.load(r)


def fetch(stem: str, mfr: str, name: str, motor_id: str) -> str:
    listing = [r for r in post("search.json", {"manufacturer": mfr, "commonName": name, "maxResults": 20})["results"]
               if r["motorId"] == motor_id]
    if not listing:
        raise RuntimeError(f"{stem}: motorId {motor_id} not found in search results")
    info = listing[0]
    files = post("download.json", {"motorIds": [motor_id], "format": "RASP", "data": "file"})["results"]
    # Pick the simfile by conformance: among curves integrating to within 1 % of
    # the listed total impulse, prefer certification data, then manufacturer,
    # then user-submitted; ties go to the smaller error.
    candidates = []
    for f in files:
        data = base64.b64decode(f["data"]).decode("utf-8", errors="replace").replace("\r\n", "\n")
        err = _impulse_error(data, info["totImpulseNs"])
        if err is not None and abs(err) <= IMPULSE_TOLERANCE:
            rank = SOURCE_PREFERENCE.index(f["source"]) if f["source"] in SOURCE_PREFERENCE else 99
            candidates.append((rank, abs(err), f["simfileId"], f, data))
    if not candidates:
        raise RuntimeError(f"{stem}: no simfile within {IMPULSE_TOLERANCE:.0%} of the listed total impulse")
    _, _, _, f, data = min(candidates, key=lambda c: c[:3])
    header = [
        f"; PROVENANCE: downloaded from the thrustcurve.org public API on {dt.date.today().isoformat()}",
        f";   motorId {motor_id}, simfile {f['simfileId']}",
        f";   (data source: \"{f['source']}\"{' = certification test data' if f['source'] == 'cert' else ''}).",
        f";   Published summary ({info.get('manufacturer')} {info.get('designation')}, {info.get('diameter')} mm):",
        f";   total impulse {info['totImpulseNs']} Ns, burn {info.get('burnTimeS')} s, "
        f"avg thrust {info.get('avgThrustN')} N, max {info.get('maxThrustN')} N,",
        f";   propellant {_g(info.get('propWeightG'))} g, loaded mass {_g(info.get('totalWeightG'))} g "
        f"(thrustcurve.org listing).",
        "; The data below is reproduced unchanged.",
    ]
    return "\n".join(header) + "\n" + data.rstrip("\n") + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args(argv)
    MOTOR_DIR.mkdir(parents=True, exist_ok=True)
    for stem, mfr, name, mid in MOTORS:
        path = MOTOR_DIR / f"{stem}.eng"
        if path.name == "Estes_C6.eng":
            continue
        if path.exists() and not args.force:
            print(f"  exists  {path.name}")
            continue
        path.write_text(fetch(stem, mfr, name, mid), encoding="utf-8")
        print(f"  wrote   {path.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
