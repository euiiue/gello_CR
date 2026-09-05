"""Hardware adapters.

Vendor SDKs stay below this boundary. UI/application code imports contracts or
these adapters, never NRC/Dynamixel/LinkerHand/RealSense SDKs directly.
"""

from .gello import GelloConfig, GelloDevice
from .o6 import O6Config, O6Device

__all__ = ["GelloConfig", "GelloDevice", "O6Config", "O6Device"]
