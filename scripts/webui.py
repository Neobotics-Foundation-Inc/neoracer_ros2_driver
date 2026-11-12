"""
Shared assets for the pages the car serves: styles, fonts, logos, icons.

Every page the car serves loads scripts/web/ under /ui/ through
static_file(), so all of them use one copy of the NeoRacer styles (the
Neobotics design system's dashboard look) and work on the access point with
no internet.
"""

from __future__ import annotations

from pathlib import Path
from urllib.parse import unquote, urlparse

WEB_DIR = Path(__file__).resolve().parent / 'web'
PREFIX = '/ui/'
# Phones refetch an updated stylesheet within minutes rather than keeping it.
CACHE_CONTROL = 'max-age=300'
# Pages link the shared files as /ui/<file>?v=ASSET_VERSION; bump it when
# neoracer.css, charts.js, theme.js or lost.js changes, so browsers fetch the
# new copy at once.
ASSET_VERSION = '1'
TYPES = {
    '.css': 'text/css; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.svg': 'image/svg+xml',
    '.png': 'image/png',
    '.webp': 'image/webp',
    '.woff2': 'font/woff2',
    '.txt': 'text/plain; charset=utf-8',
}


def static_file(request_path: str, root: Path = WEB_DIR) -> tuple[bytes, str] | None:
    """Return (body, content type) for a /ui/ path, or None when it is not one we serve."""
    path = unquote(urlparse(request_path).path)
    if not path.startswith(PREFIX):
        return None
    target = (root / path.removeprefix(PREFIX)).resolve()
    if root.resolve() not in target.parents or not target.is_file():
        return None
    kind = TYPES.get(target.suffix)
    if kind is None:
        return None
    return target.read_bytes(), kind
