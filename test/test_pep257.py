"""ament_pep257 docstring check."""

from pathlib import Path

from ament_pep257.main import main
import pytest

# The lab dashboards in scripts/dashboards/<id>/ are checked with the package. A
# car set up before they moved in-tree can still hold checkouts of their old
# repositories beside them, or in .migrated/ once setup_dashboards.sh has moved
# them; those are other repositories' code. ament_pep257 resolves excludes to
# absolute paths, so this works whether or not the directories exist.
DASHBOARDS = Path(__file__).parent.parent / 'scripts' / 'dashboards'
OLD_REPOS = ('camlabel', 'wallfollow', 'pursuit', 'eps', 'smartfollow', 'linefollow', 'teleop')
OLD_CHECKOUTS = [str(DASHBOARDS / f'{name}_dashboard') for name in OLD_REPOS] + [
    str(DASHBOARDS / '.migrated')
]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test', '--exclude', *OLD_CHECKOUTS])
    assert rc == 0, 'Found code style errors / warnings'
