import numpy as np
import matplotlib.pyplot as plt
class LinearSpline:
    def __init__(self):
        self.values = []

    def add_entry(self, t, x):
        self.values.append((t,x))

    def interpolate(self, t):
        resK  = 0
        tk, vk = self.values[0]
        #trouvons k
        for val in self.values :
                tk1,vk1 = val
                if tk1 > t:
                    resK = vk +((t-tk)/(tk1-tk))*(vk1 - vk) 
                    break
                tk,vk = val
        return resK
class LinearSpline3D:
    splinex = 0
    spliney = 0
    splinez = 0
    def __init__(self):
        self.splinex = LinearSpline()
        self.spliney = LinearSpline()
        self.splinez = LinearSpline()
    def add_entry(self, t, x, y ,z):
        self.splinex.add_entry(t,x)
        self.spliney.add_entry(t,y)
        self.splinez.add_entry(t,z)

    def interpolate(self, t):
        return self.splinex.interpolate(t), self.spliney.interpolate(t), self.splinez.interpolate(t)

if __name__ == "__main__":
    spline = LinearSpline()
    spline.add_entry(0., 0.)
    spline.add_entry(1.2, 6)
    spline.add_entry(2.5, -3)
    spline.add_entry(3.3, -2)
    spline.add_entry(4.2, -2)
    spline.add_entry(5, 0)

    xs = np.arange(-0.1,6, 0.1)
    ys = []
    for x in xs:
        ys.append(spline.interpolate(x))
    plt.plot(xs, ys)
    plt.show()
