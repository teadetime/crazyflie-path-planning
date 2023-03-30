import pytest

from path_planning import sandbox


@pytest.fixture
def simple_csv() -> str:
    """Simple CSV filepath."""
    return "tests/assets/simple.csv"


def test_main_succeeds():
    sandbox.main()
    assert True