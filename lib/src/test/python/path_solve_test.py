import json
import math
import sys
import time
import numpy
from scipy import linalg

# TEST VARS
MAX_TEST_TIME: int = 20000000 #ns
MAX_ERROR: float = 0.00001

coefficient_matrix: list[list[float]] = [
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 2, 0, 0],
    [1, 1, 1, 1, 1, 1],
    [5, 4, 3, 2, 1, 0],
    [20, 12, 6, 2, 0, 0]
]

class Segment:
    def __init__(self):
        self.delta_time: int = 0
        self.y_coeffs: numpy.array = None
        self.x_coeffs: numpy.array = None

    def compute_coeffs_for_testing(self, x_derivs, y_derivs) -> None:
        time1: int = time.perf_counter_ns()
        self.x_coeffs = linalg.solve(coefficient_matrix, x_derivs)
        self.y_coeffs = linalg.solve(coefficient_matrix, y_derivs)
        time2: int = time.perf_counter_ns()

        self.delta_time = time2 - time1


    def get_test_output_as_dict(self) -> dict:
        dict_output: dict = {
            "x": self.x_coeffs.tolist(),
            "y": self.y_coeffs.tolist(),
            "t": self.delta_time
        }

        print(json.dumps(dict_output))
        return dict_output

# main
java_data: dict = json.loads(sys.argv[1])

segment = Segment()
nparr = numpy.array(java_data["points"])
print(nparr[:, 0], nparr[:, 1])
segment.compute_coeffs_for_testing(nparr[:, 0], nparr[:, 1])
py_data: dict = segment.get_test_output_as_dict()

#if java_data["time"] > MAX_TEST_TIME:
#    print("FAIL: JAVA TEST TOOK TOO LONG")
#    exit(1)

for pair in zip(java_data["xcoef"], py_data["x"]):
    if abs(pair[0] - pair[1]) > MAX_ERROR:
        print("FAIL: RESULTS DIFFERENCE EXCEEDS MAX ERROR")
        exit(-1)

exit(0)