import tflite_runtime.interpreter as tflite
import numpy as np
from camera_capture import GetImageAsArray


class ImagePrediction:

	def __init__(self):
		self.picam = GetImageAsArray()

		self.interpreter = tflite.Interpreter(model_path="leg_clf.tflite")
		self.interpreter.allocate_tensors()

		self.input_details = self.interpreter.get_input_details()
		self.output_details = self.interpreter.get_output_details()

	def get_prediction(self):
		input_image = self.picam.get_image_array()
		input_shape = self.input_details[0]['shape']

		input_image = np.expand_dims(input_image, axis = 0)

		self.interpreter.set_tensor(self.input_details[0]['index'], input_image)
		self.interpreter.invoke()

		output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
		result = round(output_data[0][0])

		return result
