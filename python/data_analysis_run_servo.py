import numpy as np
import pandas
import matplotlib.pyplot as plt

with open("data\\5.6\\srvotst.csv") as f:
    servo_pd = pandas.read_csv(f)
    f.close()

print(servo_pd.head())
plt.title("Y vs U for SG995 Left Hip")
plt.scatter(servo_pd["Time"], servo_pd["Left_Hip"], color = "tab:red", label="Y_Left")
plt.scatter(servo_pd["Time"], servo_pd["Right_Hip"], color = "tab:green", label="Y_Right")
plt.scatter(servo_pd["Time"], servo_pd["U"], color = "tab:blue", label="U")
plt.ylabel("Servo angle Y (degrees)")
plt.xlabel("Time (ms)")

plt.legend()
plt.show()