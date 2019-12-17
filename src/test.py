import math
import numpy as np
from functions import Euler2A
from functions import A2AxisAngle
from functions import Rodrigez
from functions import A2Euler
from functions import AngleAxis2Q
from functions import Q2AxisAngle


def test(fi, teta, psi):
    print("Polazni Ojlerovi uglovi: ")
    print("Fi ~ " + str(round(fi / math.pi * 180)) + " stepeni")
    print("Teta ~ " + str(round(teta / math.pi * 180)) + " stepeni")
    print("Psi ~ " + str(round(psi / math.pi * 180)) + " stepeni")
    print()

    A = Euler2A(fi, teta, psi)
    print("Matrica A = Rz(psi)Ry(teta)Rx(fi):")
    print(A)
    print()

    p, alpha = A2AxisAngle(A)
    print("Rotacija oko prave p: ")
    print(p)
    print("za ugao: ")
    print("Psi ~ " + str(round(alpha / math.pi * 180)) + " stepeni")
    print()

    B = Rodrigez(p, alpha)
    print("Rodrigezovom formulom dobija se matrica: ")
    print(B)
    print()

    beta, gamma, delta = A2Euler(B)
    print("Uglovi dobijeni iz matrice dobijene Rodrigezovom formulom: ")
    print("Beta ~ " + str(round(beta / math.pi * 180)) + " stepeni")
    print("Gamma ~ " + str(round(gamma / math.pi * 180)) + " stepeni")
    print("Delta ~ " + str(round(delta / math.pi * 180)) + " stepeni")
    print()

    q = AngleAxis2Q(p, alpha)
    print("Kvaternion koji predstavlja rotaciju: (zaokruzen na 6 decimala)")
    print([round(k, 6) for k in q])
    print()

    p, fi = Q2AxisAngle(q)
    print("Kvaternion predstavlja rotaciju oko prave: ")
    print(p)
    print("za ugao:")
    print("~ " + str(round(fi / math.pi * 180)) + " stepeni")
    print()


def main():
    print("TEST SA SAJTA")
    test(-math.atan(1 / 4), -math.asin(8 / 9), math.atan(4))
    print()

    print("TEST1")
    A = np.array([[3, 1, math.sqrt(6)],
                  [1, 3, -math.sqrt(6)],
                  [-math.sqrt(6), math.sqrt(6), 2]])
    A *= 0.25
    fi, delta, psi = A2Euler(A)
    test(fi, delta, psi)
    print()

    print("TEST2")
    A = np.array([[0, 0, -1],
                  [0, -1, 0],
                  [-1, 0, 0]])
    fi, delta, psi = A2Euler(A)

    test(fi, delta, psi)
    print()

    print("TEST3")
    test(0, math.pi/4, 0)
    print()


# if __name__ == '__main__':
#     main()
