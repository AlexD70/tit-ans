import sys
import json
import matplotlib.pyplot as plt
import numpy

java_data: dict = json.loads(sys.argv[1])

x = [j[0] for j in java_data["points"]]
y = [j[1] for j in java_data["points"]]
plt.plot(numpy.array(x), numpy.array(y), 'o')
plt.show()

exit(0)
