import random
from itertools import count
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#plt.style.use('fivethirtyeight')
matplotlib.use('TkAgg')

x_vals = []
y_vals = []

index = count()
fig, (ax1, ax2) = plt.subplots(2)

def animate(i):
    data = pd.read_csv('data.csv')
    x = data['time']
    y1 = data['x']
    y2 = data['y']
    y3 = data['z']
    # y4 = data['rx']
    # y5 = data['ry']
    # y6 = data['rz']
    y7 = data['xf']
    #y8 = data['yf']
    #y9 = data['zf']
    # y10 = data['rxf']
    # y11 = data['ryf']
    # y12 = data['rzf']
    status_x = data['statusx']
    status_y = data['statusy']
    status_z = data['statusz']


    ax1.cla()
    ax2.cla()

    ax1.plot(x, y1, label='Coord x')
    #ax1.plot(x, y2, label='Channel y', color='green')
    #ax1.plot(x, y3, label='Channel z', color='blue')
    # ax1.plot(x, y4, label='Channel rx')
    # ax1.plot(x, y5, label='Channel ry')
    # ax1.plot(x, y6, label='Channel rz')
    ax1.plot(x, y7, label='Coord x after Kalman')
    #ax1.plot(x, y8, label='Channel yf')
    # ax1.plot(x, y9, label='Channel zf')
    # ax1.plot(x, y10, label='Channel rxf')
    # ax1.plot(x, y11, label='Channel ryf')
    # ax1.plot(x, y12, label='Channel rzf')
    ax2.plot(x, status_x, label='STD', color='red')
    #ax2.plot(x, status_y, label='status_y', color='green')
    #ax2.plot(x, status_z, label='status_z', color='blue')

    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')


ani = FuncAnimation(fig, animate, interval=0.1)

plt.tight_layout()
plt.show()