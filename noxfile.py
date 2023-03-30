"""Nox sessions."""

import nox
from nox_poetry import Session

python_version = "3.10"
nox.options.sessions = "lint", "tests", "pyright"


@nox.session(python=[python_version])
def tests(session: Session):
    """Run tests with Pytest."""
    args = session.posargs
    session.install("pytest-mock")
    session.run("poetry", "install", external=True)
    session.run("pytest", *args)


locations = "src", "tests", "noxfile.py"


@nox.session(python=[python_version])
def lint(session: Session):
    """Lint using flake8."""
    args = session.posargs or locations
    session.install(
        "flake8",
        "flake8-annotations",
        "flake8-black",
        "flake8-bugbear",
        "flake8-docstrings",
        "flake8-import-order",
        "darglint",
    )
    session.run("flake8", *args)


@nox.session(python=python_version)
def black(session: Session):
    """Format using Black."""
    args = session.posargs or locations
    session.install("black")
    session.run("black", *args)


@nox.session(python=python_version)
def pyright(session: Session) -> None:
    """Type checking using pyright."""
    args = session.posargs or locations
    session.install("pyright", "pytest", "pytest-mock")
    session.run_always("poetry", "install", external=True)
    session.run("pyright", *args)
