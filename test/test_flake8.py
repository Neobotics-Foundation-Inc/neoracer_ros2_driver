"""ament_flake8 lint check."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # The lab dashboards in `dashboards` keep their own style and are not yet
    # held to this package's lint rules.
    rc, errors = main_with_errors(argv=['--exclude=build,install,log,dashboards'])
    assert rc == 0, '\n'.join(errors)
