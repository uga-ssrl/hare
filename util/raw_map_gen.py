import csv
import sys
import os
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
# import seaborn as sns

filepath = sys.argv[1]

data = list(csv.reader(open(filepath)))
jaxWins = []
i = 0
for x in data:
    jaxWins.append([])
    for y in x:
        jaxWins[i].append(int(y))
    i += 1

H = np.array(jaxWins)  # added some commas and array creation code

fig = plt.figure(figsize=(6, 3.2))

ax = fig.add_subplot(111)
ax.set_title('colorMap')
plt.imshow(H)
ax.set_aspect('equal')
plt.show()

# ax = sns.heatmap(doot, linewidth=0.5)
#plt.imshow(doot, cmap='hot')
#plt.show()













































# yeet
