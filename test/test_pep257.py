"""ament_pep257 docstring check."""

from pathlib import Path

from ament_pep257.main import main
import pytest

# Upstream lab-dashboard checkouts live here; they are third-party code and are
# not held to this package's docstring convention. ament_pep257 resolves
# excludes to absolute paths, so this works whether or not the directory exists.
DASHBOARDS = str(Path(__file__).parent.parent / 'scripts' / 'dashboards')


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test', '--exclude', DASHBOARDS])
    assert rc == 0, 'Found code style errors / warnings'
