"""Keystone: M-01 SENTINEL's safety kernel composed with a real UR5e LeRobot driver.

The driver in this package carries no safety logic. Safety is sentinel.Shield's
job; keystone moves the arm and reports its state.
"""
from __future__ import annotations

from keystone.config import UR5eConfig

__version__ = "0.1.0"

__all__ = ["UR5eConfig", "__version__"]
