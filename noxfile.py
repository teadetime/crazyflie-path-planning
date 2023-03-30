# noxfile.py
import nox
from nox_poetry import Session

python_version = "3.10"
nox.options.sessions = "lint", "tests", "pyright"


@nox.session(python=[python_version])
def tests(session: Session):
    args = session.posargs
    session.install("pytest-mock")
    session.run("poetry", "install", external=True)
    session.run("pytest", *args)


locations = "src", "tests", "noxfile.py"


@nox.session(python=[python_version])
def lint(session: Session):
    args = session.posargs or locations
    session.install(
        "flake8",
        "flake8-annotations",
        "flake8-black",
        "flake8-bugbear",
        "flake8-import-order",
    )
    session.run("flake8", *args)


@nox.session(python=python_version)
def black(session: Session):
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
