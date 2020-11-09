import numpy as np
import pandas
import matplotlib.pyplot as plt

with open("python/data/servo_dat2.csv") as f:
    servo_pd = pandas.read_csv(f)
    f.close()

print(servo_pd.head())
servo_pd["Time"] = servo_pd["Time"] - servo_pd["Time"][0]
print(servo_pd.head())
plt.scatter(servo_pd["Time"], servo_pd["Left"], label="Left theta")
#plt.scatter(servo_pd["Time"], servo_pd["Right"], label="Right theta")

grad = np.gradient(servo_pd["Left"], servo_pd["Time"])
gradgrad = np.gradient(grad, servo_pd["Time"])
plt.scatter(servo_pd["Time"], grad * 1000, label="Left thetad")
plt.scatter(servo_pd["Time"], gradgrad * 1000, label="Left thetadd")
plt.legend()
plt.show()