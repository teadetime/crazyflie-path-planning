

Getting started developing:

1. Install Nox

    `pip install --upgrade nox`

2. Install [Poetry](https://python-poetry.org/docs/)

    `curl -sSL https://install.python-poetry.org | python3 -`

3. Install Python ^3.10 (recommended via `pyenv`)

4. Clone the 

    `git clone git@github.com:teadetime/crazyflie-path-planning.git`

5. Within the repo

    `poetry install`

6. Run nox

    `nox -r` 

    to run linting: `nox -rs lint`

    to run static type-checking: `nox -rs pyright`

    to run tests: `nox -rs tests`

    to run black(formatter): `nox -rs black`

7. Run the sandbox file

    `poetry run sandbox` or ` poetry run python3 src/path_planning/sandbox.py`