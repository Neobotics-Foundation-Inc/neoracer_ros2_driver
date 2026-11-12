"""Shared web assets: scripts/webui.py and scripts/web/."""

from pathlib import Path
import re
from xml.etree import ElementTree

from conftest import load_script
import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'


@pytest.fixture(scope='module')
def webui():
    return load_script('webui')


def test_serves_the_stylesheet(webui):
    body, kind = webui.static_file('/ui/neoracer.css')
    assert kind.startswith('text/css') and b'--nr-red' in body


def test_serves_fonts_logos_scripts_and_icons(webui):
    assert webui.static_file('/ui/fonts/space-grotesk-var.woff2')[1] == 'font/woff2'
    assert webui.static_file('/ui/brand/neoracer-logo.svg')[1] == 'image/svg+xml'
    assert webui.static_file('/ui/brand/garage.webp')[1] == 'image/webp'
    assert webui.static_file('/ui/theme.js?v=1')[1].startswith('text/javascript')
    assert webui.static_file('/ui/icons.svg?v=2')[1] == 'image/svg+xml'


@pytest.mark.parametrize(
    'path',
    [
        '/index.html',
        '/ui/',
        '/ui/missing.css',
        '/ui/../webui.py',
        '/ui/%2e%2e/webui.py',
        '/ui/fonts/../../dashboard.py',
        '/ui/../health.html',
    ],
)
def test_refuses_anything_else(webui, path):
    assert webui.static_file(path) is None


def test_refuses_unknown_types(webui, tmp_path):
    (tmp_path / 'notes.md').write_text('x')
    assert webui.static_file('/ui/notes.md', root=tmp_path) is None


def test_every_font_face_file_ships_with_its_licence(webui):
    css = (webui.WEB_DIR / 'neoracer.css').read_text()
    faces = re.findall(r'url\("(fonts/[^"]+)"\)', css)
    assert faces == ['fonts/space-grotesk-var.woff2', 'fonts/jetbrains-mono-var.woff2']
    for name in faces:
        assert (webui.WEB_DIR / name).is_file(), name
    for license_file in ('OFL-spacegrotesk.txt', 'OFL-jetbrainsmono.txt'):
        text = (webui.WEB_DIR / 'fonts' / license_file).read_text()
        assert 'SIL OPEN FONT LICENSE Version 1.1' in text, license_file


def test_stylesheet_uses_the_neobotics_values(webui):
    css = (webui.WEB_DIR / 'neoracer.css').read_text().upper()
    # red, the dash-dark and dash-light page, panel and edge, Success, Warning
    for value in (
        '#FF0033',
        '#09090B',
        '#131317',
        '#26262D',
        '#EDEDF0',
        '#D9D9DE',
        '#22C55E',
        '#FFC847',
    ):
        assert value in css, value


def test_no_upstream_brand_and_no_licensed_face_under_web(webui):
    # Upstream's colour tokens, its fonts and its name, and the design system's
    # licensed title face, never ship in this public repository.
    for path in webui.WEB_DIR.rglob('*'):
        if path.suffix in ('.css', '.js', '.svg', '.txt'):
            text = path.read_text().lower()
            for banned in ('--rcn-', 'barlow', 'kernel', 'racecar neo'):
                assert banned not in text, (path.name, banned)
    assert not list(webui.WEB_DIR.rglob('*barlow*'))
    assert not list(webui.WEB_DIR.rglob('*kernel*'))


def test_logos_are_files_with_no_script(webui):
    brand = webui.WEB_DIR / 'brand'
    for name in (
        'neoracer-logo.svg',
        'neoracer-logo-reversed.svg',
        'neoracer-logo-white.svg',
        'n-mark.svg',
        'n-mark-white.svg',
    ):
        text = (brand / name).read_text().lower()
        assert ElementTree.fromstring(text).tag.endswith('svg'), name
        assert '<script' not in text and 'href="http' not in text, name


def test_icons_parse_and_have_ids(webui):
    root = ElementTree.parse(webui.WEB_DIR / 'icons.svg').getroot()
    ids = {el.get('id') for el in root if el.tag.endswith('symbol')}
    assert {'map', 'gamepad', 'pulse', 'check', 'alert', 'gauge', 'bolt', 'battery'} <= ids


def test_pages_link_shared_files_with_the_asset_version(webui):
    # A browser keeps /ui/ files for CACHE_CONTROL; the version query is what
    # makes it fetch a changed neoracer.css or charts.js at once.
    for page in (SCRIPTS / 'health.html',):
        text = page.read_text()
        for name in ('neoracer.css', 'charts.js', 'theme.js', 'lost.js'):
            for link in re.findall(rf'"/ui/{re.escape(name)}[^"]*"', text):
                assert link == f'"/ui/{name}?v={webui.ASSET_VERSION}"', (page.name, link)


def test_the_health_page_shows_the_logo_for_each_theme():
    text = (SCRIPTS / 'health.html').read_text()
    assert 'class="for-dark" src="/ui/brand/neoracer-logo-reversed.svg"' in text
    assert 'class="for-light" src="/ui/brand/neoracer-logo.svg"' in text
    assert 'data-theme-pick="dark"' in text and 'data-theme-pick="light"' in text


def test_the_dashboard_serves_ui_health_and_host():
    text = (SCRIPTS / 'dashboard.py').read_text()
    assert 'webui.static_file(self.path)' in text
    # Routes compare the path without its query, so /health?theme=light works.
    assert 'route = urlparse(self.path).path' in text
    assert "route == '/health'" in text and "route == '/api/host'" in text
    assert "data['battery'] = _battery_host()" in text


def test_the_health_page_is_titled_and_shows_the_battery():
    text = (SCRIPTS / 'health.html').read_text()
    assert '<title>System Health · NeoRacer</title>' in text
    assert '<h1 class="nr-title">System Health</h1>' in text
    assert 'id="s-batt"' in text and 'batteryBox(batt)' in text
    # The page shows the car, not a sample of every shared piece.
    assert 'Pieces' not in text and 'nr-toast' not in text
