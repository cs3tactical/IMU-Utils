import numpy as np
import matplotlib.pyplot as plt

a = np.linspace(0, 1000, 1000000)[1:]
b = a**(-2) + a**(-1) + 1 + a**(1) + a**(2)

plt.grid(True)
plt.loglog(a, np.sqrt(b), '-')
plt.loglog(a, np.sqrt(np.array([np.mean(b)]*np.size(b))), '-')
plt.show()
