import matplotlib.pyplot as plt
import numpy as np
fig = plt.figure()
ax: plt.Axes = fig.subplots()  # type: ignore

# Time difference between sensor sample publish and sample receive in detector,
# before and after orchestrator was running

before = np.genfromtxt("times_baseline.csv")
after = np.genfromtxt("times_forwarded.csv")
bins = np.arange(0.0, 0.003, 0.00005)
ax.hist(before, bins=bins, label="before")
ax.hist(after, bins=bins, label="after")

mean = np.mean(before)
mean_after = np.mean(after)

ax.legend()
ax.set_title(f"Mean before: {mean:.6f}s, after: {mean_after:.6f}s")
plt.show()
