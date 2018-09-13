#!/usr/bin/env python

import sys
import math
from PySide2 import QtCore, QtGui, QtWidgets, QtOpenGL
from PySide2.QtCore import Signal, Slot
import rospy
from std_msgs.msg import String
import threading
from datetime import datetime

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
        self.ros_thread = threading.Thread(target=self.glWidget.listener, name="listener")
        self.glWidget.setListenerThread(self.ros_thread)
        self.ros_thread.start()


class GLWidget(QtOpenGL.QGLWidget):
    xRotationChanged = QtCore.Signal(int)
    yRotationChanged = QtCore.Signal(int)
    conn = QtCore.Signal(str)

    def __init__(self, parent=None):
        QtOpenGL.QGLWidget.__init__(self, parent)

        self.object = 0
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0
        self.xPos = 0
        self.yPos = 0
        self.zPos = -10
        self.scale = 1

        self.lastPos = QtCore.QPoint()

        self.conn.connect(self.redraw)

        self.trolltechGreen = QtGui.QColor.fromCmykF(0.40, 0.0, 1.0, 0.0)
        self.trolltechPurple = QtGui.QColor.fromCmykF(0.39, 0.39, 0.0, 0.0)

    def setListenerThread(self, thread):
        self.listThread = thread

    @Slot(str)
    def redraw(self, data):
        #Model Changing
        if("torus" in data):
                print("changing to torus")
                self.object = self.torus()
                self.updateGL()
        if("cuboid" in data):
                print("changing to cuboid")
                self.object = self.cuboid()
                self.updateGL()
        if("sphere" in data):
                print("changing to sphere")
                self.object = self.sphere()
                self.updateGL()
        if("cylinder" in data):
                print("changing to cylinder")
                self.object = self.cylinder()
                self.updateGL()
        if("rotate" in data):
                grade = 30
	        #Rotations
                if("up" in data):
                    print("Rotating {} degrees on +Y direction".format(grade))
                    self.setXRotation(self.xRot - grade)
                if("down" in data):
                    print("Rotating {} degrees on -Y direction".format(grade))
                    self.setXRotation(self.xRot + grade)
                if("left" in data):
                    print("Rotating {} degrees on +X direction".format(grade))
                    self.setYRotation(self.yRot - grade)
                if("right" in data):
                    print("Rotating {} degrees on -X direction".format(grade))
                    self.setYRotation(self.yRot + grade)
        if("translate" in data):
                step = 1
	        #Rotations
                if("up" in data):
                    print("Translate {} step on +Y direction".format(step))
                    self.setYTranslation(self.yPos + step/100)
                if("down" in data):
                    print("Translate {} step on -Y direction".format(step))
                    self.setYTranslation(self.yPos - step/100)
                if("left" in data):
                    print("Translate {} step on +X direction".format(step))
                    self.setXTranslation(self.xPos - step/100)
                if("right" in data):
                    print("Translate {} step on -X direction".format(step))
                    self.setXTranslation(self.xPos + step/100)
        if("zoom" in data):
                if("in" in data):
                    print("Zooming in the object")
                    self.setScale(self.scale + 0.05)
                if("out" in data):
                    print("Zooming out the object")
                    self.setScale(self.scale - 0.05)
        if("reset" in data):
                print("reseting view")
                self.resetView()
        if("snapshot" in data):
                print("taking a snapshot")
                self.snapshot(datetime.now().strftime("%Y-%m-%d.%H:%M:%S"))
        if("exit" in data):
                print("closing application")

    def callback(self,data):
        self.conn.emit(data.data)

    def listener(self):
    	print("Starting qt_app listener")
    	rospy.init_node('listener', anonymous=True, disable_signals=True)
    	rospy.Subscriber('qt_app', String, self.callback)
    	rospy.spin()

    def minimumSizeHint(self):
        return QtCore.QSize(50, 50)

    def sizeHint(self):
        return QtCore.QSize(400, 400)

    def snapshot(self, filename, fileformat='png'):
        g = self.geometry()
        fg = self.frameGeometry()
        rfg = fg.translated(-g.left(),-g.top())
        pixmap =  QtGui.QPixmap.grabWindow(self.winId(),rfg.left(), rfg.top(),rfg.width(), rfg.height())
        pixmap.save(filename+"."+fileformat, fileformat)

    def resetView(self):
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0
        self.xPos = 0
        self.yPos = 0
        self.zPos = -10
        self.scale = 1
        self.updateGL()

    def setScale(self, scale):
        if(scale < 0.7):
            scale = 0.7
        if(scale > 1.3):
            scale = 1.3
        self.scale = scale
        self.updateGL()

    def setXTranslation(self, move):
        if(move < -.25):
            move = -.25
        if(move > .25):
            move = .25
        if move != self.xPos:
            self.xPos = move
            self.updateGL()
    
    def setYTranslation(self, move):
        if(move < -.25):
            move = -.25
        if(move > .25):
            move = .25
        if move != self.yPos:
            self.yPos = move
            self.updateGL()

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

    def initializeGL(self):
        self.qglClearColor(self.trolltechPurple.darker())
        self.object = self.cuboid()
        GL.glShadeModel(GL.GL_FLAT)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_CULL_FACE)
		
    def sphere(self, r = .35, lats = 100, longs = 100):
        sphereList = GL.glGenLists( 1 )
        GL.glNewList( sphereList, GL.GL_COMPILE )
        GL.glColor3f(1,1,1);
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

                GL.glNormal3d(x * zr0 * r, y * zr0 * r, z0 * r)				
                GL.glVertex3d(x * zr0 * r, y * zr0 * r, z0 * r)
                GL.glNormal3d(x * zr1 * r, y * zr1 * r, z1 * r)
                GL.glVertex3d(x * zr1 * r, y * zr1 * r, z1 * r)
            GL.glEnd()
        GL.glEndList()
        return sphereList

    def cuboid(self, side = .25):
        cubeList = GL.glGenLists( 1 )
        GL.glNewList( cubeList, GL.GL_COMPILE )
        GL.glBegin(GL.GL_QUADS);
		
        GL.glColor3f(0,1,0);		
        GL.glNormal3f( 1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f( 1.0*side, 1.0*side,-1.0*side)
        GL.glNormal3f(-1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side,-1.0*side)
        GL.glNormal3f(-1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side, 1.0*side)
        GL.glNormal3f( 1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side, 1.0*side, 1.0*side)

        GL.glColor3f(1,0,0);
        GL.glNormal3f( 1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side,-1.0*side, 1.0*side)
        GL.glNormal3f(-1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side, 1.0*side)
        GL.glNormal3f(-1.0*side,-1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side,-1.0*side)
        GL.glNormal3f( 1.0*side,-1.0*side,-1.0*side)
        GL.glVertex3f( 1.0*side,-1.0*side,-1.0*side)

        GL.glColor3f(0,0,1);		
        GL.glNormal3f( 1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side, 1.0*side, 1.0*side)
        GL.glNormal3f(-1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side, 1.0*side)
        GL.glNormal3f(-1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side, 1.0*side)
        GL.glNormal3f( 1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side,-1.0*side, 1.0*side)

        GL.glColor3f(1,0,1);
        GL.glNormal3f( 1.0*side,-1.0*side,-1.0*side)
        GL.glVertex3f( 1.0*side,-1.0*side,-1.0*side)
        GL.glNormal3f(-1.0*side,-1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side,-1.0*side)
        GL.glNormal3f(-1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side,-1.0*side)
        GL.glNormal3f( 1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f( 1.0*side, 1.0*side,-1.0*side)

        GL.glColor3f(1,1,0);		
        GL.glNormal3f(-1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side, 1.0*side)
        GL.glNormal3f(-1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side, 1.0*side,-1.0*side)
        GL.glNormal3f(-1.0*side,-1.0*side,-1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side,-1.0*side)
        GL.glNormal3f(-1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f(-1.0*side,-1.0*side, 1.0*side)

        GL.glColor3f(0,1,1);
        GL.glNormal3f( 1.0*side, 1.0*side,-1.0*side)
        GL.glVertex3f( 1.0*side, 1.0*side,-1.0*side)
        GL.glNormal3f( 1.0*side, 1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side, 1.0*side, 1.0*side)
        GL.glNormal3f( 1.0*side,-1.0*side, 1.0*side)
        GL.glVertex3f( 1.0*side,-1.0*side, 1.0*side)
        GL.glNormal3f( 1.0*side,-1.0*side,-1.0*side)
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
            GL.glNormal3f(x,y,height/2)
            GL.glVertex3f(x,y,height/2)
            GL.glNormal3f(x,y,-height/2)
            GL.glVertex3f(x,y,-height/2)
            angle += ang_step
        GL.glNormal3f(radius, 0, height/2)
        GL.glVertex3f(radius, 0, height/2)
        GL.glNormal3f(radius, 0, -height/2)
        GL.glVertex3f(radius, 0, -height/2)
        GL.glEnd()
		
        GL.glColor3f(0,1,0);
        GL.glBegin(GL.GL_POLYGON)
        angle = 0
        while(angle < 2*math.pi):
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            GL.glNormal3f(x,y,height/2)
            GL.glVertex3f(x,y,height/2)
            angle += ang_step
        GL.glNormal3f(radius,0,height/2)
        GL.glVertex3f(radius,0,height/2)
        GL.glEnd()
		
        GL.glColor3f(0,1,0);
        GL.glBegin(GL.GL_POLYGON)
        angle = 0
        while(angle < 2*math.pi):
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            GL.glNormal3f(x,y,-height/2)
            GL.glVertex3f(x,y,-height/2)
            angle += ang_step
        GL.glNormal3f(radius,0,-height/2)
        GL.glVertex3f(radius,0,-height/2)
        GL.glEnd()
        GL.glEndList()
        return cylList
	
    def torus(self, rin = 0.07, rout = 0.2, sides = 100, rings = 20):
        torusList = GL.glGenLists( 1 )
        GL.glNewList( torusList, GL.GL_COMPILE )
        GL.glColor3f(1,1,1);		
        
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
                GL.glNormal3d((rout+rr*math.cos(v))*math.cos(w),(rout+rr*math.cos(v))*math.sin(w),rin*math.sin(v))
                GL.glVertex3d((rout+rr*math.cos(v))*math.cos(w),(rout+rr*math.cos(v))*math.sin(w),rin*math.sin(v))
                GL.glNormal3d((rout+rr*math.cos(v+dv))*math.cos(w+dw),(rout+rr*math.cos(v+dv))*math.sin(w+dw),rin*math.sin(v+dv))
                GL.glVertex3d((rout+rr*math.cos(v+dv))*math.cos(w+dw),(rout+rr*math.cos(v+dv))*math.sin(w+dw),rin*math.sin(v+dv))
                v += dv
            GL.glEnd()
            w += dw		
		
        GL.glEndList()
        return torusList

    def paintGL(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glLoadIdentity()
        GL.glScalef(self.scale,self.scale,self.scale)
        GL.glTranslated(self.xPos, self.yPos, self.zPos)
        GL.glRotated(self.xRot, 1.0, 0.0, 0.0)
        GL.glRotated(self.yRot, 0.0, 1.0, 0.0)
        GL.glRotated(self.zRot, 0.0, 0.0, 1.0)
        GL.glCallList(self.object)

    def resizeGL(self, width, height):
        side = min(width, height)
        GL.glViewport(int((width - side) / 2),int((height - side) / 2), side, side)

        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 360
        while angle > 360:
            angle -= 360
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
