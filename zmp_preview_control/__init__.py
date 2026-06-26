"""ZMP Preview Control — Walking Pattern Generator for Biped Robots."""

__version__ = "2.0.0"

from .params import get_preview_control_parameter
from .simulation import create_zmp_trajectory, calc_preview_control, simulate
