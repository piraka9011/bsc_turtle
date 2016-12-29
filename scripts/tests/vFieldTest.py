from matplotlib.pyplot import cm
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    '''
    X, Y = np.mgrid[-3:3:15j, -3:3:15j]
    U = Y
    V = -X
    speed = np.sqrt(U**2 + V**2)
    UN = U/speed
    VN = V/speed

    plot1 = plt.figure()
    plt.quiver(X, Y, U, V,
               U,
               cmap=cm.seismic,
               headlength=7)
    plt.colorbar()
    plt.title("Test1")
    plt.show(plot1)
    '''
    plot2 = plt.figure()
    x = np.linspace(-5.5, 2.5, 5)
    gridY = [-5, -2, 0, 2]
    X, Y = np.meshgrid(x, gridY)
    plt.scatter(X, Y)
    plt.grid()
    plt.title("Test2")
    plt.show(plot2)