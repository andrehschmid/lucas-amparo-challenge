# lucas-amparo-challenge

Build an QT 5 (https://www.qt.io/) application that has a 3D viewer component.

In that 3D viewer, the user must be able to select a geometry to render: a torus, a cylinder, a cuboid or a sphere, at any time.

The 3D viewer camera must be controlled by commands published in a single ROS Topic, which will publish some information based on the keyboard input:
* Move up, down, left and right;
* Rotate up, down, left and right;
* Zoom in and out.

The QT application must be a ROS client to read that topic.
The user must be able to take a screen shot of that 3D view, that must be saved on the disk. In other words, the 3D view must be converted to a 2D image based on the camera view position.

The solution can be developed with Python or C++.
Specially in the QT part, it must be implemented using QML (http://doc.qt.io/qt-5/qtqml-index.html) along with Python (https://www.qt.io/qt-for-python) or C++.

You must push your code regularly to this git repository.

Here is a simple architecture diagram to ilustrate the solution. Feel free to use another architecture if you find it better.

![Architecture Image](https://www.dropbox.com/s/0dyhyzwwalgrkur/QT%20ROS%20architecture.png?raw=1)