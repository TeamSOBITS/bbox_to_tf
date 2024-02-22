import matplotlib.pyplot as plt

width=14
height=10

plt.cla()
plt.plot([0, 0], [0, height], "-b")
plt.plot([width, width], [0, height], "-b")
plt.plot([0, width], [0, 0], "-b")
plt.plot([0, width], [height, height], "-b")

plt.xlim([-7,width + 9])
plt.ylim([-6,height + 3])
plt.pause(0.3)

for w in range(width//2):
    for h in range(height//2):
        plt.plot([width//2-w],[height//2-h],"or")
        plt.pause(0.01)
        if (h != 0):
            plt.plot([width//2-w],[height//2+h],"or")
            plt.pause(0.01)
        if (w == 0):
            continue
        plt.plot([width//2+w],[height//2-h],"or")
        plt.pause(0.01)
        if (h != 0):
            plt.plot([width//2+w],[height//2+h],"or")
            plt.pause(0.01)
plt.show()