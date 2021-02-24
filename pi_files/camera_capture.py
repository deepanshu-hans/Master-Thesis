from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np

class GetImageAsArray:

	def __init__(self, resolution = 128):
		self.camera = PiCamera()
		self.camera.resolution = (resolution, resolution)

	def get_image_array(self):
		time.sleep(0.1)
		rawCapture = PiRGBArray(self.camera)
		self.camera.capture(rawCapture, format = 'rgb')
		image = rawCapture.array
		return image.astype(np.float32)


