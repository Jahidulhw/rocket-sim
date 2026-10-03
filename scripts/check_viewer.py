"""Smoke-test the web viewer in a headless Chromium browser (Chrome or Edge).

    python scripts/check_viewer.py [--browser PATH] [--screenshots DIR]

For every dataset in web/data/index.json this serves web/ on a local port,
loads the page headlessly and checks the DOM state the viewer exposes:
    body[data-status]   == "ready"   data loaded and the scene was built
    body[data-rendered] == "true"    at least one frame was rendered
    body[data-errors]   == "0"       no uncaught errors / console.error calls
Then it serves a copy of the site whose index points at a missing file and
checks that the viewer shows its visible error message instead of failing
silently.

Needs network access to cdn.jsdelivr.net (three.js). Exits non-zero on any
failure; prints SKIP and exits 0 if no browser is found. This is a dev check,
not part of the pytest suite, because it depends on a local browser.
"""

from __future__ import annotations

import argparse
import functools
import html
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
WEB = REPO_ROOT / "web"

CANDIDATES = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "google-chrome", "chromium", "chromium-browser", "microsoft-edge",
]


def find_browser(explicit: str | None) -> str | None:
    for c in [explicit, os.environ.get("BROWSER")] + CANDIDATES:
        if not c:
            continue
        if Path(c).exists():
            return c
        found = shutil.which(c)
        if found:
            return found
    return None


class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


def serve(directory: Path) -> ThreadingHTTPServer:
    handler = functools.partial(_QuietHandler, directory=str(directory))
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def run_browser(browser: str, url: str, profile: Path, screenshot: Path | None = None) -> str:
    args = [browser, "--headless=new", "--no-first-run", "--no-default-browser-check",
            "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--hide-scrollbars",
            f"--user-data-dir={profile}", "--window-size=1280,800", "--virtual-time-budget=15000"]
    if screenshot is not None:
        args.append(f"--screenshot={screenshot}")
    else:
        args.append("--dump-dom")
    args.append(url)
    out = subprocess.run(args, capture_output=True, text=True, encoding="utf-8", errors="replace",
                         timeout=120)
    return out.stdout


def body_attrs(dom: str) -> dict:
    m = re.search(r"<body([^>]*)>", dom)
    if not m:
        return {}
    return {k: html.unescape(v) for k, v in re.findall(r'data-([\w-]+)="([^"]*)"', m.group(1))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--browser")
    ap.add_argument("--screenshots", help="also save a screenshot per dataset into this directory")
    args = ap.parse_args(argv)

    browser = find_browser(args.browser)
    if not browser:
        print("SKIP: no Chrome/Edge/Chromium found (use --browser PATH)")
        return 0
    index = json.loads((WEB / "data" / "index.json").read_text(encoding="utf-8"))
    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        profile = Path(tmp) / "profile"
        server = serve(WEB)
        base = f"http://127.0.0.1:{server.server_address[1]}/"
        print(f"Browser: {browser}\nServing {WEB} at {base}")
        for ds in index["datasets"]:
            attrs = body_attrs(run_browser(browser, f"{base}?data={ds['id']}&t=5", profile))
            ok = (attrs.get("status") == "ready" and attrs.get("rendered") == "true"
                  and attrs.get("errors") == "0")
            failures += not ok
            print(f"  [{'PASS' if ok else 'FAIL'}] {ds['id']:<12} status={attrs.get('status')} "
                  f"rendered={attrs.get('rendered')} errors={attrs.get('errors')} "
                  f"{'' if ok else attrs.get('error-log', '')}")
            if args.screenshots:
                Path(args.screenshots).mkdir(parents=True, exist_ok=True)
                run_browser(browser, f"{base}?data={ds['id']}&t=5", profile,
                            Path(args.screenshots).resolve() / f"{ds['id']}.png")
        server.shutdown()

        # Failure path: a copy of the site whose dataset file is missing.
        broken = Path(tmp) / "web"
        shutil.copytree(WEB, broken, ignore=shutil.ignore_patterns("*.json"))
        (broken / "data").mkdir(exist_ok=True)
        (broken / "data" / "index.json").write_text(json.dumps(
            {"datasets": [{"id": "missing", "kind": "flight", "label": "Missing", "file": "missing.json"}]}))
        server = serve(broken)
        dom = run_browser(browser, f"http://127.0.0.1:{server.server_address[1]}/", profile)
        server.shutdown()
        attrs = body_attrs(dom)
        err_box = re.search(r'<div id="error"([^>]*)>(.*?)</div>', dom, re.S)
        visible = bool(err_box) and "hidden" not in err_box.group(1) and "HTTP 404" in err_box.group(2)
        ok = attrs.get("status") == "error" and visible
        failures += not ok
        print(f"  [{'PASS' if ok else 'FAIL'}] missing-data shows visible error: "
              f"{html.unescape(err_box.group(2)).strip()[:80] if err_box else None!r}")

    print("Viewer check:", "OK" if failures == 0 else f"{failures} FAILURE(S)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
