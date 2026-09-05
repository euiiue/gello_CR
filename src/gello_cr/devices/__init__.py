"""Hardware adapters.

Vendor SDKs stay below this boundary.  UI/application code imports contracts or
these adapters, never NRC/Dynamixel/LinkerHand/RealSense SDKs directly.
"""

from .gello import GelloConfig, GelloDevice

__all__ = ["GelloConfig", "GelloDevice"]
