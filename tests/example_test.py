from unittest.mock import Mock

import pytest
from pytest_mock import MockFixture

from path_planning import sandbox


@pytest.fixture
def simple_csv() -> str:
    """Simple CSV filepath."""
    return "tests/assets/simple.csv"


@pytest.fixture
def mock_requests_get(mocker: MockFixture) -> Mock:
    return Mock()


def test_main_succeeds() -> None:
    print("Within a test")
    sandbox.main(5)
    assert True
