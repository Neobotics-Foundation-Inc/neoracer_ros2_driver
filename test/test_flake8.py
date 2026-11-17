"""ament_flake8 lint check."""

from ament_flake8.main import main_with_errors
import pytest

# The lab dashboards in scripts/dashboards/<id>/ are linted with the package. A
# car set up before they moved in-tree can still hold checkouts of their old
# repositories beside them, or in .migrated/ once setup_dashboards.sh has moved
# them; those are other repositories' code.
OLD_REPOS = ('camlabel', 'wallfollow', 'pursuit', 'eps', 'smartfollow', 'linefollow', 'teleop')
OLD_CHECKOUTS = ','.join([f'{name}_dashboard' for name in OLD_REPOS] + ['.migrated'])


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[f'--exclude=build,install,log,{OLD_CHECKOUTS}'])
    assert rc == 0, '\n'.join(errors)
