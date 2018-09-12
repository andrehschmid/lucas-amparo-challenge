#!/usr/bin/env python

import sys
import math
from PySide2 import QtCore, QtGui, QtWidgets, QtOpenGL

try:
    from OpenGL import GL
except ImportError:
    app = QtWidgets.QApplication(sys.argv)
    messageBox = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, "Lucas Amparo :: Challenge",
                                       "PyOpenGL must be installed to run this example.",
                                       QtWidgets.QMessageBox.Close)
    messageBox.setDetailedText("Run:\npip install PyOpenGL PyOpenGL_accelerate")
    messageBox.exec_()
    sys.exit(1)


class Window(QtWidgets.QWidget):
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)

        self.glWidget = GLWidget()

        mainLayout = QtWidgets.QHBoxLayout()
        mainLayout.addWidget(self.glWidget)
        self.setLayout(mainLayout)
		
        self.setWindowTitle(self.tr("Lucas Amparo :: Challenge"))


class GLWidget(QtOpenGL.QGLWidget):
    xRotationChanged = QtCore.Signal(int)
    yRotationChanged = QtCore.Signal(int)
    zRotationChanged = QtCore.Signal(int)

    def __init__(self, parent=None):
        QtOpenGL.QGLWidget.__init__(self, parent)

        self.object = 0
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0

        self.lastPos = QtCore.QPoint()

        self.trolltechGreen = QtGui.QColor.fromCmykF(0.40, 0.0, 1.0, 0.0)
        self.trolltechPurple = QtGui.QColor.fromCmykF(0.39, 0.39, 0.0, 0.0)

    def xRotation(self):
        return self.xRot

    def yRotation(self):
        return self.yRot

    def zRotation(self):
        return self.zRot

    def minimumSizeHint(self):
        return QtCore.QSize(50, 50)

    def sizeHint(self):
        return QtCore.QSize(400, 400)

    def setXRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.xRot:
            self.xRot = angle
            self.emit(QtCore.SIGNAL("xRotationChanged(int)"), angle)
            self.updateGL()

    def setYRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.yRot:
            self.yRot = angle
            self.emit(QtCore.SIGNAL("yRotationChanged(int)"), angle)
            self.updateGL()

    def setZRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.zRot:
            self.zRot = angle
            self.emit(QtCore.SIGNAL("zRotationChanged(int)"), angle)
            self.updateGL()

    def initializeGL(self):
        self.qglClearColor(self.trolltechPurple.darker())
        self.object = self.torus()
        GL.glShadeModel(GL.GL_FLAT)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_CULL_FACE)
		
    def sphere(self, r = .35, lats = 100, longs = 100):
        sphereList = GL.glGenLists( 1 )
        GL.glNewList( sphereList, GL.GL_COMPILE )
        for i in range(lats):
            lat0 = math.pi * (-0.5 + (i-1)/lats)
            z0 = math.sin(lat0)
            zr0 = math.cos(lat0)
			
            lat1 = math.pi * (-0.5 + i/lats)
            z1 = math.sin(lat1)
            zr1 = math.cos(lat1)
			
            GL.glBegin(GL.GL_POINTS)
            for j in range(longs):
                lng = 2 * math.pi * (j-1) / longs
                x = math.cos(lng)
                y = math.sin(lng)
				
                GL.glVertex3d(x * zr0 * r, y * zr0 * r, z0 * r)
                GL.glVertex3d(x * zr1 * r, y * zr1 * r, z1 * r)
            GL.glEnd()
        GL.glEndList()
        return sphereList

    def cuboid(self, side = .25):
        cubeList = GL.glGenLists( 1 )
        GL.glNewList( cubeList, GL.GL_COMPILE )
        GL.glBegin(GL.GL_QUADS);
		
        GL.glColor3f(0,1,0);		
        GL.glVertex3f( 1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side, 1.0*side, 1.0*side)

        GL.glColor3f(1,0,0);				
        GL.glVertex3f( 1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side,-1.0*side)
        GL.glVertex3f( 1.0*side,-1.0*side,-1.0*side)

        GL.glColor3f(0,0,1);		
        GL.glVertex3f( 1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side,-1.0*side, 1.0*side)

        GL.glColor3f(1,0,0);
        GL.glVertex3f( 1.0*side,-1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f( 1.0*side, 1.0*side,-1.0*side)

        GL.glColor3f(0,1,0);		
        GL.glVertex3f(-1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side, 1.0*side)

        GL.glColor3f(0,0,1);
        GL.glVertex3f( 1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f( 1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side,-1.0*side,-1.0*side)
		
        GL.glEnd()
        GL.glEndList()
        return cubeList
		
    def cylinder(self, radius = .2, height = .5):
        cylList = GL.glGenLists( 1 )
        GL.glNewList( cylList, GL.GL_COMPILE )
        x = 0
        y = 0
        angle = 0
        ang_step = 0.1
		
        GL.glColor3f(0,0,1);
        GL.glBegin(GL.GL_QUAD_STRIP)
        angle = 0
        while(angle < 2*math.pi):
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            GL.glVertex3f(x,y,height/2)
            GL.glVertex3f(x,y,-height/2)
            angle += ang_step
        GL.glVertex3f(radius, 0, height/2)
        GL.glVertex3f(radius, 0, -height/2)
        GL.glEnd()
		
        GL.glColor3f(0,1,0);
        GL.glBegin(GL.GL_POLYGON)
        angle = 0
        while(angle < 2*math.pi):
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            GL.glVertex3f(x,y,height/2)
            angle += ang_step
        GL.glVertex3f(radius,0,height)
        GL.glEnd()
		
        GL.glColor3f(0,1,0);
        GL.glBegin(GL.GL_POLYGON)
        angle = 0
        while(angle < 2*math.pi):
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            GL.glVertex3f(x,y,-height/2)
            angle += ang_step
        GL.glVertex3f(radius,0,-height/2)
        GL.glEnd()
        GL.glEndList()
        return cylList
	
    def torus(self, rin = 0.07, rout = 0.2, sides = 100, rings = 20):
        torusList = GL.glGenLists( 1 )
        GL.glNewList( torusList, GL.GL_COMPILE )
        
        #Drawing the torus
        rr = 1.5*rin
        dv = 2*math.pi/rings
        dw = 2*math.pi/sides
		
        v = 0
        w = 0
		
        while(w < 2*math.pi+dw):
            v = 0
            GL.glBegin(GL.GL_TRIANGLE_STRIP)
            while(v < 2*math.pi+dv):
                GL.glVertex3d((rout+rr*math.cos(v))*math.cos(w),(rout+rr*math.cos(v))*math.sin(w),rin*math.sin(v))
                GL.glVertex3d((rout+rr*math.cos(v+dv))*math.cos(w+dw),(rout+rr*math.cos(v+dv))*math.sin(w+dw),rin*math.sin(v+dv))
                v += dv
            GL.glEnd()
            w += dw		
		
        GL.glEndList()
        return torusList

    def paintGL(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glLoadIdentity()
        GL.glTranslated(0.0, 0.0, -10.0)
        GL.glRotated(self.xRot / 16.0, 1.0, 0.0, 0.0)
        GL.glRotated(self.yRot / 16.0, 0.0, 1.0, 0.0)
        GL.glRotated(self.zRot / 16.0, 0.0, 0.0, 1.0)
        GL.glCallList(self.object)

    def resizeGL(self, width, height):
        side = min(width, height)
        GL.glViewport(int((width - side) / 2),int((height - side) / 2), side, side)

        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)

    def mousePressEvent(self, event):
        self.lastPos = QtCore.QPoint(event.pos())

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if event.buttons() & QtCore.Qt.LeftButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setYRotation(self.yRot + 8 * dx)
        elif event.buttons() & QtCore.Qt.RightButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setZRotation(self.zRot + 8 * dx)

        self.lastPos = QtCore.QPoint(event.pos())

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 360 * 16
        while angle > 360 * 16:
            angle -= 360 * 16
        return angle

    def freeResources(self):
        self.makeCurrent()
        GL.glDeleteLists(self.object, 1)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    res = app.exec_()
    window.glWidget.freeResources()
    sys.exit(res)
