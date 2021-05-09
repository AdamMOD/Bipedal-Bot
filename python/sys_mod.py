import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate

xr = np.linspace(-90,90,1000)
taylor = scipy.interpolate.approximate_taylor_polynomial(np.tanh, 0, 10, 90)

plt.plot(xr, np.tanh(xr))
plt.plot(xr, taylor(xr))
plt.ylim(-2,2)
plt.show()