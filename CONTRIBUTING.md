

Getting started developing:

1. Install Nox

    `pip install --upgrade nox`

2. Install [Poetry](https://python-poetry.org/docs/)

    `curl -sSL https://install.python-poetry.org | python3 -`

3. Install pre-commit for precommit hooks and nox-poetry

    `pip install --upgrade pre-commit`
    `pip install nox-poetry`

4. Install Python ^3.10 (recommended via `pyenv`)

5. Clone the [repo](https://github.com/teadetime/crazyflie-path-planning)

    `git clone git@github.com:teadetime/crazyflie-path-planning.git`

6. Within the repo

    `poetry install`

7. Install the pre-commit hook and test it

    `pre-commit install`


    `pre-commit run --all-files`

6. Run nox

    `nox -r`

    to run linting: `nox -rs lint`

    to run static type-checking: `nox -rs pyright`

    to run tests: `nox -rs tests`

    to run black(formatter): `nox -rs black`

7. Run the sandbox file

    `poetry run sandbox` or ` poetry run python3 src/path_planning/sandbox.py`
