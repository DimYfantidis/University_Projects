# tflite_api/__init__.py
from .TFliteAPI import TFLiteManager
# Added Quantization Aware stuff
from .QuantizationAwareInterface import *

__version__ = "2.1"
__author__ = "Dimitrios Yfantidis"
print(f"{__name__} v{__version__} package initialized")
