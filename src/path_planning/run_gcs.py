import numpy as np

from path_planning.algorithms.gcs import GCS

from path_planning.omap import OMap
from path_planning.utils import Agent, Goal
from .utils import Point

def main() -> None:
    gcs = GCS.generate(OMap(), Goal(np.array([0, -5, 0])))