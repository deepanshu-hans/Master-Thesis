import tflite_runtime.interpreter as tflite
import numpy as np
from laser_scan import Lidar


class LidarPrediction:

	def __init__(self):
		self.lidar = Lidar(150)

		self.interpreter = tflite.Interpreter(model_path="leg_clf_lidar.tflite")
		self.interpreter.allocate_tensors()

		self.input_details = self.interpreter.get_input_details()
		self.output_details = self.interpreter.get_output_details()

	def normalize(distances):
		distances = [val if val > 0.01 else 1.0 for val in distances]
		distances = [val if val < 1.0 else 1.0 for val in distances]
		return np.array(distances)

	def get_prediction(self):
		distance = self.lidar.get_distance()
		input_instance = LidarPrediction.normalize(distance)
		input_shape = self.input_details[0]['shape']
		input_instance = np.expand_dims(input_instance, axis=0).astype(np.float32)
		self.interpreter.set_tensor(self.input_details[0]['index'], input_instance)
		self.interpreter.invoke()

		output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
		output_data = (list(map(np.float32, output_data[0])))
		result = output_data.index(max(output_data))
		return result, input_instance[0]
