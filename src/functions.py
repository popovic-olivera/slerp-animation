import math
import sys
import numpy as np
import numpy.linalg as la


# Rz(psi)*Ry(teta)*Rx(fi)
def Euler2A(fi, teta, psi):
    sin_fi = math.sin(fi)
    cos_fi = math.cos(fi)
    sin_teta = math.sin(teta)
    cos_teta = math.cos(teta)
    sin_psi = math.sin(psi)
    cos_psi = math.cos(psi)

    return np.array([
        [cos_teta*cos_psi, cos_psi*sin_teta*sin_fi-cos_fi*sin_psi, cos_fi*cos_psi*sin_teta+sin_fi*sin_psi],
        [cos_teta*sin_psi, cos_fi*cos_psi+sin_teta*sin_fi*sin_psi, cos_fi*sin_teta*sin_psi-cos_psi*sin_fi],
        [-sin_teta, cos_teta*sin_fi, cos_teta*cos_fi]
    ])


def A2AxisAngle(A):
    if (A == np.eye(3)).all() or round(la.det(A)) != 1:
        sys.exit("A2AxisAngle: Matrica A je jedinicna ili joj je determinanta razlicita od 1...")

    # Odredjujemo jedinicni sopstveni vektor za lambda = 1
    # A*p = lambda*p => 0 = (A - E)p

    A_p = A - np.eye(3)

    u = []
    p = []

    # p ce biti jednak vektorskom proizvodu dve ne nula vrste matrice A_p
    # (jer p treba da bude normalan na sve vrste, sto se vidi iz toga da je skalarni proizvod = 0)
    # posto u treba da bude normalan na p, moze se uzeti bilo koja ne nula vrsta matrice A_p

    if not np.array_equal(np.round(np.cross(A_p[0], A_p[1]), 3), np.zeros((3,))):
        p = np.cross(A_p[0], A_p[1])
        u = np.array(A_p[0])
    elif not np.array_equal(np.round(np.cross(A_p[1], A_p[2]), 3), np.zeros((3,))):
        p = np.cross(A_p[1], A_p[2])
        u = np.array(A_p[1])
    elif not np.array_equal(np.round(np.cross(A_p[2], A_p[0]), 3), np.zeros((3,))):
        p = np.cross(A_p[2], A_p[0])
        u = np.array(A_p[2])

    # postavimo da p i u budu jedinicni
    norm_p = np.linalg.norm(p)
    if norm_p != 0:
        p = p / norm_p

    norm = np.linalg.norm(u)
    if norm != 0:
        u = u / norm

    # ne mora se proveravati da li je jedinicni jer je A izometrija
    u_prim = A.dot(u)

    alpha = math.acos(u.dot(u_prim))

    # proveravamo da bi rotacija bila u pozitivnom smeru
    if (np.cross(u, u_prim)).dot(p) < 0:
        p = -p

    return p, alpha


# dobija se matrica rotacije za ugao alpha oko orijentisane prave p
def Rodrigez(p, alpha):
    if round(la.norm(p)) != 1:
        norm_p = la.norm(p)
        if norm_p != 0:
            p = p / norm_p

    # Formula Rodrigeza:
    # Rp(fi) = p*p_t + cos(fi)*(E - p*p_t) + sin(fi)*px

    pp_t = np.outer(p, p)
    cos_alpha = math.cos(alpha)
    sin_alpha = math.sin(alpha)

    px = np.array([
        [0, -p[2], p[1]],
        [p[2], 0, -p[0]],
        [-p[1], p[0], 0]
    ])

    return pp_t + cos_alpha*(np.eye(3) - pp_t) + sin_alpha*px


def A2Euler(A):
    if round(la.det(A)) != 1:
        sys.exit("Matrica B nije ortogonalna...")

    A = np.array(A)
    if A[2, 0] < 1:
        if A[2, 0] > -1:
            psi = math.atan2(A[1, 0], A[0, 0])
            delta = math.asin(-A[2, 0])
            fi = math.atan2(A[2, 1], A[2, 2])
        else:   # nije jedinstveno resenje
            psi = math.atan2(-A[0, 1], A[1, 1])
            delta = math.pi / 2     # A[2, 0] = sin(delta) = 1 tj. delta = pi/2
            fi = 0
    else:   # nije jedinstveno resenje
        psi = math.atan2(-A[0, 1], A[1, 1])
        delta = -math.pi / 2    # A[2, 0] = -sin(delta) = 1 tj. delta = -pi/2
        fi = 0

    return fi, delta, psi


# svaki q = [vector*sin(alpha), cos(alpha)] je svetska rotacija oko vector za 2*alpha
def AngleAxis2Q(p, fi):
    if fi == 0:
        return 1

    w = math.cos(fi / 2)

    norm_p = la.norm(p)
    if norm_p != 0:
        p = p / norm_p

    x, y, z = math.sin(fi / 2) * p

    return [x, y, z, w]


def Q2AxisAngle(q):
    norm_q = la.norm(q)
    if norm_q != 0:
        q = q / norm_q

    w = q[3]

    # ako zelimo da fi bude iz [0, pi]
    if w < 0:
        q = -q

    fi = 2 * math.acos(w)

    if abs(w) == 1:
        p = (1, 0, 0)   # bilo koji jedinicni
    else:
        xyz = np.array([q[0], q[1], q[2]])
        norm = la.norm(xyz)

        if norm != 0:
            p = xyz / norm
        else:
            p = xyz

    return p, fi


def slerp(q1, q2, tm, t):
    if t < 0 or t > tm:
        sys.exit("t nije u odgovarajucem opsegu")

    norm_q1 = la.norm(q1)
    if norm_q1 != 0:
        q1 = q1 / norm_q1

    norm_q2 = la.norm(q2)
    if norm_q2 != 0:
        q2 = q2 / norm_q2

    cos0 = np.dot(np.array(q1), np.array(q2))

    if cos0 < 0:
        q1 = -q1
        cos0 = -cos0

    if cos0 > 0.95:
        return q1

    fi0 = math.acos(cos0)

    return ((math.sin(fi0 * (1 - t/tm))) / math.sin(fi0))*q1 + ((math.sin(fi0 * (t/tm))) / math.sin(fi0))*q2

