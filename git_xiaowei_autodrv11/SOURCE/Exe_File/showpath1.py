import matplotlib.pyplot as plt
import numpy as np

#x1 = [0 for _ in range(3459)]
#y1 = [0 for _ in range(3459)]
x2 = [0 for _ in range(250)]
y2 = [0 for _ in range(250)]

def loadtxtmethod(filename):
    data = np.loadtxt(filename,dtype=np.float,delimiter=',')
    return data
 
if __name__=="__main__":
    #data = loadtxtmethod('gnss_data.txt')
    #for i in range(0, len(data), 4):
        #x1[i/4] = data[i]
        #y1[i/4] = data[i+1]

    data = loadtxtmethod('gps_load.txt')
    for i in range(0, len(data), 4):
        x2[i/4] = data[i]
        y2[i/4] = data[i+1]


    # print(x) 
    # print(y)
    plt.plot(x2, y2,'go')
    # plt.plot(x, y)
    plt.show()
         
