#from path_solve_test import *
import sys

import scipy.integrate as sciintegr
import math

from scipy import optimize


class QuinticPolynomial:
    def __init__(self, *args):
        if len(args) == 6:
            self.a = args[0]
            self.b = args[1]
            self.c = args[2]
            self.d = args[3]
            self.e = args[4]
            self.f = args[5]
        elif len(args[0]) == 6:
            self.a = args[0][0]
            self.b = args[0][1]
            self.c = args[0][2]
            self.d = args[0][3]
            self.e = args[0][4]
            self.f = args[0][5]
        else:
            raise ValueError(
                "Invalid polynomial coefficients given. Expected either six separate coefficients or an array of six coefficients.")

    def eval(self, t):
        return self.a * t**5 + self.b * t**4 + self.c * t**3 + self.d * t**2 + self.e * t + self.f

    def first_deriv(self):
        return QuinticPolynomial(0, 5 * self.a, 4 * self.b, 3 * self.c, 2 * self.d, self.e)

    def second_deriv(self):
        return QuinticPolynomial(0, 0, 20 * self.a, 12 * self.b, 6 * self.c, 2 * self.d)

xpoly = QuinticPolynomial(0, 0, 0, 0, 1, -2.3)
ypoly = QuinticPolynomial(0, 0, 0, -4, 1, 4)

def displacement_at_parameter(t):
    length, _ = sciintegr.quad(unit_arc_length, 0, t)
    return length

def parameter_at_displacement(s0) -> float:
    def f(t):
        return displacement_at_parameter(t) - s0

    t = optimize.brentq(f, 0, 1)
    return t


def unit_arc_length(tau):
    return math.sqrt(xpoly.first_deriv().eval(tau)**2 + ypoly.first_deriv().eval(tau)**2)


if __name__ == "__main__":
    print(parameter_at_displacement(1))
