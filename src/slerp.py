from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *

import math
from functions import Euler2A
from functions import A2AxisAngle
from functions import AngleAxis2Q
from functions import Q2AxisAngle
from functions import slerp


class Globals:
    def __init__(self):
        self.timer_active = False
        self.t = 0
        self.tm = 40
        self.q1 = []
        self.q2 = []
        self.fi1 = math.pi/6
        self.teta1 = 3*math.pi/4
        self.psi1 = -math.pi/2
        self.fi2 = math.pi/2
        self.teta2 = math.pi/4
        self.psi2 = 2*math.pi/3
        self.x1 = -5
        self.y1 = 0
        self.z1 = 5
        self.x2 = 5
        self.y2 = 3
        self.z2 = -2


g = Globals()


def onDisplay():
    global g

    # parametri svetla
    light_position = [0, 7, 0, 0]
    light_ambient = [0.5, 0.5, 0.5, 0.1]
    light_diffuse = [0.8, 0.8, 0.8, 1]
    light_specular = [0.6, 0.6, 0.6, 1]
    shininess = 30

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # podesava se tacka pogleda
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(8, 10, 8,
              0, 0, 0,
              0, 1, 0)

    # objekti zadrzavaju boju
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    # podesava se svetlo
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)

    glLightfv(GL_LIGHT0, GL_POSITION, light_position)
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse)
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular)
    glMaterialf(GL_FRONT, GL_SHININESS, shininess)

    # Iscrtava se koordinatni sistem
    drawCoordinateSystem(12)

    # Iscrtavaju se pocetni i krajnji objekat
    drawBeginAndEnd(g.x1, g.y1, g.z1, g.fi1, g.teta1, g.psi1, 0, 0.75, 1)
    drawBeginAndEnd(g.x2, g.y2, g.z2, g.fi2, g.teta2, g.psi2, 0, 0.75, 1)

    # Iscrtava se pomereni objekat u trenutku t
    drawAnimated(0, 0.75, 1)

    glutSwapBuffers()


def drawCoordinateSystem(size):
    glBegin(GL_LINES)
    glColor3f(1, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(size, 0, 0)

    glColor3f(0, 1, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, size, 0)

    glColor3f(0, 0, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, size)
    glEnd()


def drawBeginAndEnd(x, y, z, fi, teta, psi, r, g, b):
    glPushMatrix()

    glColor3f(r, g, b)
    glTranslatef(x, y, z)

    A = Euler2A(fi, teta, psi)
    p, alpha = A2AxisAngle(A)

    glRotatef(alpha / math.pi * 180, p[0], p[1], p[2])

    glutSolidTeapot(1.5)
    drawCoordinateSystem(4)

    glPopMatrix()


def drawAnimated(r, gr, b):
    global g

    glPushMatrix()
    glColor3f(r, gr, b)

    x = (1 - g.t/g.tm)*g.x1 + (g.t/g.tm)*g.x2
    y = (1 - g.t / g.tm) * g.y1 + (g.t / g.tm) * g.y2
    z = (1 - g.t / g.tm) * g.z1 + (g.t / g.tm) * g.z2

    glTranslatef(x, y, z)

    q = slerp(g.q1, g.q2, g.tm, g.t)
    p, fi = Q2AxisAngle(q)

    glRotatef(fi / math.pi * 180, p[0], p[1], p[2])

    glutSolidTeapot(1.5)

    drawCoordinateSystem(4)

    glPopMatrix()


def onReshape(w, h):
    glViewport(0, 0, w, h)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60, float(w) / h, 1, 1500)


def onKeyboard(ch, x, y):
    global g

    if ord(ch) == 27:
        sys.exit(0)
    elif ord(ch) == ord('g') or ord(ch) == ord('G'):
        if not g.timer_active:
            glutTimerFunc(100, onTimer, 0)
            g.timer_active = True
    elif ord(ch) == ord('s') or ord(ch) == ord('S'):
        g.timer_active = False


def onTimer(value):
    global g

    if value != 0:
        return

    g.t += 1
    if g.t > g.tm:
        g.t = 0
        g.timer_active = False
        return

    glutPostRedisplay()

    if g.timer_active:
        glutTimerFunc(100, onTimer, 0)


def main():
    global g

    glutInit()
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)
    glutInitWindowSize(800, 600)
    glutInitWindowPosition(0, 0)
    glutCreateWindow("SLERP animacija")

    glutDisplayFunc(onDisplay)
    glutReshapeFunc(onReshape)
    glutKeyboardFunc(onKeyboard)

    glEnable(GL_DEPTH_TEST)

    # inicijalizuje se pocetni kvaternion q1
    A = Euler2A(g.fi1, g.teta1, g.psi1)
    p, alpha = A2AxisAngle(A)
    g.q1 = AngleAxis2Q(p, alpha)

    # inicijalizuje se krajnji kvaternion q2
    A = Euler2A(g.fi2, g.teta2, g.psi2)
    p, alpha = A2AxisAngle(A)
    g.q2 = AngleAxis2Q(p, alpha)

    glutMainLoop()


if __name__ == '__main__':
    main()
