"""Camera package"""

__all__ = ['CameraHAL', 'OpenCVCamera', 'SyntheticCamera', 'create_camera']


def __getattr__(name):
    # capture는 OpenCV가 필요하므로 실제로 쓸 때만 import (recorder는 OpenCV 불필요)
    if name in __all__:
        from . import capture
        return getattr(capture, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
