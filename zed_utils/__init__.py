# zed_utils/__init__.py

# Expose the main class so it can be imported as:
# from zed_utils import ZEDBagReader
from .zed_bag_reader import ZEDBagReader

# Define the version of your wrapper
__version__ = "1.0.0"

# Explicitly define what is exported when someone uses 'from zed_utils import *'
__all__ = ["ZEDBagReader"]