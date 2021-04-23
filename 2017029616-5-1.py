import glfw
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import math
def drawFrame(): 
    glBegin(GL_LINES) 
    glColor3ub(255, 0, 0) 
    glVertex3fv(np.array([0.,0.,0.]))
    glVertex3fv(np.array([1.,0.,0.])) 
    glColor3ub(0, 255, 0)
    glVertex3fv(np.array([0.,0.,0.])) 
    glVertex3fv(np.array([0.,1.,0.])) 
    glColor3ub(0, 0, 255)
    glVertex3fv(np.array([0.,0.,0])) 
    glVertex3fv(np.array([0.,0.,1.]))
    glEnd()

def drawUnitCube(): 
    glBegin(GL_QUADS)
    glVertex3f( 0.5, 0.5,-0.5)
    glVertex3f(-0.5, 0.5,-0.5)
    glVertex3f(-0.5, 0.5, 0.5)
    glVertex3f( 0.5, 0.5, 0.5)
    glVertex3f( 0.5,-0.5, 0.5)
    glVertex3f(-0.5,-0.5, 0.5)
    glVertex3f(-0.5,-0.5,-0.5) 
    glVertex3f( 0.5,-0.5,-0.5)
    glVertex3f( 0.5, 0.5, 0.5)
    glVertex3f(-0.5, 0.5, 0.5) 

    glVertex3f(-0.5,-0.5, 0.5)
    glVertex3f( 0.5,-0.5, 0.5)
    glVertex3f( 0.5,-0.5,-0.5)
    glVertex3f(-0.5,-0.5,-0.5)
    glVertex3f(-0.5, 0.5,-0.5)
    glVertex3f( 0.5, 0.5,-0.5)
    glVertex3f(-0.5, 0.5, 0.5)
    glVertex3f(-0.5, 0.5,-0.5)
    glVertex3f(-0.5,-0.5,-0.5)
    glVertex3f(-0.5,-0.5, 0.5)
    glVertex3f( 0.5, 0.5,-0.5)
    glVertex3f( 0.5, 0.5, 0.5)
    glVertex3f( 0.5,-0.5, 0.5)
    glVertex3f( 0.5,-0.5,-0.5)
    glEnd()

def drawCubeArray():
 for i in range(5): 
    for j in range(5): 
        for k in range(5): 
            glPushMatrix( )
            glTranslatef(i,j,-k-1) 
            glScalef(.5,.5,.5) 
            drawUnitCube()
            glPopMatrix()

def render(): 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) 
    glEnable(GL_DEPTH_TEST) 
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE ) 
    glLoadIdentity() 
    myOrtho(-5,5, -5,5, -8,8) 
    myLookAt(np.array([5,3,5]), np.array([1,1,-1]), np.array([0,1,0])) 
    # Above two lines must behaves exactly same as the below two lines 
    #glOrtho(-5,5, -5,5, -8,8) 
    #gluLookAt(5,3,5, 1,1,-1, 0,1,0) 
    drawFrame() 
    glColor3ub(255, 255, 255) 
    drawCubeArray()
    
def myOrtho(left, right, bottom, top, near, far): 
    glMultMatrixf(np.array([[2/(right-left), 0, 0, -1*((right+left)/(right-left))],
                    [0, 2/(top-bottom), 0 , -1*((top+bottom)/(top-bottom))],
                    [0, 0, 2/(near-far), -1*((near+far)/(near-far))],
                    [0, 0, 0, 1]]).T)
def myLookAt(eye, at, up):
    z = eye-at
    z = z/ np.sqrt(np.dot(z,z))
    x = np.cross(up, z)
    x = x/ np.sqrt(np.dot(x,x))
    y = np.cross(z, x)
    pos = np.array([-1*np.dot(eye,x), -1*np.dot(eye,y), -1*np.dot(eye,z)])
    glMultMatrixf(np.array([[x[0], y[0], z[0], 0],
                  [x[1], y[1], z[1], 0],
                  [x[2], y[2], z[2], 0],
                  [pos[0], pos[1], pos[2], 1]]))
    '''
    glTranslate(eye[0], eye[1], eye[2])
    lookvec = at - eye
    arr =np.cross(np.array([0,0,-1]), eye)
    temp = np.dot(np.array([0,0,-1]),arr)
    temp = temp / np.sqrt(np.dot(arr, arr))
    angle = math.acos(temp)
    glRotatef(0, arr[0], arr[1], arr[2])
    '''

def main():
    if not glfw.init():
        return
    window = glfw.create_window(1000, 480, "2017029616-5-1", None, None)
    if not window:
        glfw.terminate()
        return
    glfw.make_context_current(window)
    
    while not glfw.window_should_close(window):
        glfw.poll_events()
        render()
        glfw.swap_buffers(window)
    glfw.terminate()
    
if __name__ == "__main__":
    main()
