import matplotlib.pyplot as plt

w=13
h=10

plt.cla()
plt.xlim([0,w])
plt.ylim([0,h])

plt.plot([w//2],[h//2],"or")
plt.pause(0.5)

for i in range(w//2):
    for j in range(h//2):
        if ((i==0) and (j==0)):
            continue
        plt.plot([w//2-i],[h//2-j],"or")
        plt.pause(0.5)
        if (j!=0):
            plt.plot([w//2-i],[h//2+j],"or")
            plt.pause(0.5)
        if (i==0):
            continue
        plt.plot([w//2+i],[h//2-j],"or")
        plt.pause(0.5)
        if (j!=0):
            plt.plot([w//2+i],[h//2+j],"or")
            plt.pause(0.5)
plt.show()