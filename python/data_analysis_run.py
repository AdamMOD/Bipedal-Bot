import numpy as np
import pandas
import matplotlib.pyplot as plt

with open("data\\v2run4.28\\balanceatteptstartthenbug.csv") as f:
    run = pandas.read_csv(f)
    f.close()


plt.title("Y vs U for SG995 Left Hip")
plt.scatter(run["Time"], run["Command_0"], color = "tab:red", label="Command 0")
plt.scatter(run["Time"], run["Pitch"], color = "tab:blue", label="Pitch")
plt.scatter(run["Time"], run["Pitch_Rate"], color = "tab:green", label="Pitch_Rate")

plt.legend()
plt.show()