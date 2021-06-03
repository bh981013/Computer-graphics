import glfw
import sys
import pdb
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.arrays import ArrayDatatype
import time
import numpy as np
import ctypes
from PIL.Image import open
import OBJ
from Ray import *
import copy
import math
import time
cow2wld_list=[]
temp_cow2wld=[]
cownum = 0

# global variables
wld2cam=[]
cam2wld=[]
#cow2wld=None
cursorOnCowBoundingBox=False
pickInfo=None
floorTexID=0
currentPos = []
cameras= [
	[28,18,28, 0,2,0, 0,1,0],   
	[28,18,-28, 0,2,0, 0,1,0], 
	[-28,18,28, 0,2,0, 0,1,0], 
	[-12,12,0, 0,2,0, 0,1,0],  
	[0,100,0,  0,0,0, 1,0,0]
]
camModel=None
cowModel=None
H_DRAG=1
V_DRAG=2
# dragging state
isDrag=0
quit = 0




class PickInfo:
    def __init__(self, cursorRayT, cowPickPosition, cowPickConfiguration, cowPickPositionLocal):
        self.cursorRayT=cursorRayT
        self.cowPickPosition=cowPickPosition.copy()
        self.cowPickConfiguration=cowPickConfiguration.copy()
        self.cowPickPositionLocal=cowPickPositionLocal.copy()

def vector3(x,y,z):
    return np.array((x,y,z))

def position3(v):
    # divide by w
    w=v[3]
    return vector3(v[0]/w, v[1]/w, v[2]/w)

def vector4(x,y,z):
    return np.array((x,y,z,1))

def rotate(m,v):
    return m[0:3, 0:3]@v

def transform(m, v):
    return position3(m@np.append(v,1))

def getTranslation(m):
    return m[0:3,3]
def setTranslation(m,v):
    m[0:3,3]=v

def makePlane( a,  b,  n):
    v=a.copy()
    for i in range(3):
        if n[i]==1.0:
            v[i]=b[i];
        elif n[i]==-1.0:
            v[i]=a[i];
        else:
            assert(n[i]==0.0);
            
    return Plane(rotate(cow2wld_list[cownum],n),transform(cow2wld_list[cownum],v));

def onKeyPress( window, key, scancode, action, mods):
    global cameraIndex, quit
    if action==glfw.RELEASE:
        return ; # do nothing
    if key==glfw.KEY_C or key==glfw.KEY_SPACE:
        print( "Toggle camera %s\n"% cameraIndex );
        cameraIndex += 1;
    '''
    if key==glfw.KEY_Q:
        quit = 1
    if key == glfw.KEY_S:
        for i in cowpos:
            print(i)
    '''
    if cameraIndex >= len(wld2cam):
        cameraIndex = 0;

def drawOtherCamera():
    global cameraIndex,wld2cam, camModel
    for i in range(len(wld2cam)):
        if (i != cameraIndex):
            glPushMatrix();												
            glMultMatrixd(cam2wld[i].T)
            drawFrame(5);											
            frontColor = [0.2, 0.2, 0.2, 1.0];
            glEnable(GL_LIGHTING);									
            glMaterialfv(GL_FRONT, GL_AMBIENT, frontColor);			
            glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);			
            glScaled(0.5,0.5,0.5);										
            glTranslated(1.1,1.1,0.0);									
            camModel.render()
            glPopMatrix();												

def drawFrame(leng):
    glDisable(GL_LIGHTING);	# Lighting is not needed for drawing axis.
    glBegin(GL_LINES);		# Start drawing lines.
    glColor3d(1,0,0);		# color of x-axis is red.
    glVertex3d(0,0,0);			
    glVertex3d(leng,0,0);	# Draw line(x-axis) from (0,0,0) to (len, 0, 0). 
    glColor3d(0,1,0);		# color of y-axis is green.
    glVertex3d(0,0,0);			
    glVertex3d(0,leng,0);	# Draw line(y-axis) from (0,0,0) to (0, len, 0).
    glColor3d(0,0,1);		# color of z-axis is  blue.
    glVertex3d(0,0,0);
    glVertex3d(0,0,leng);	# Draw line(z-axis) from (0,0,0) - (0, 0, len).
    glEnd();			# End drawing lines.

def drawCow(_cow2wld, drawBB, i):

    global doAni

    glPushMatrix();		

    
    glMultMatrixd(_cow2wld.T)
    drawFrame(5);										# Draw x, y, and z axis.
    frontColor = [0.8, 0.2, 0.9, 1.0];
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT, GL_AMBIENT, frontColor);		# Set ambient property frontColor.
    glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);		# Set diffuse property frontColor.
    cowModel.render()	# Draw cow. 
    glDisable(GL_LIGHTING);
    if drawBB and i == cownum and doAni == 0: #i'th cow is the last cow, which should draw surrounding boxes
        glBegin(GL_LINES);
        glColor3d(1,1,1);
        cow=cowModel
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);

        glColor3d(1,1,1);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);

        glColor3d(1,1,1);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);


        glColor3d(1,1,1);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);

        glColor3d(1,1,1);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmin[1], cow.bbmax[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmin[0], cow.bbmax[1], cow.bbmax[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmin[2]);
        glVertex3d( cow.bbmax[0], cow.bbmax[1], cow.bbmax[2]);
        glEnd();
    glPopMatrix();			
def drawFloor():

    glDisable(GL_LIGHTING);

    # Set color of the floor.
    # Assign checker-patterned texture.
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, floorTexID );

    # Draw the floor. Match the texture's coordinates and the floor's coordinates resp. 
    nrep=4
    glBegin(GL_POLYGON);
    glTexCoord2d(0,0);
    glVertex3d(-12,-0.1,-12);		# Texture's (0,0) is bound to (-12,-0.1,-12).
    glTexCoord2d(nrep,0);
    glVertex3d( 12,-0.1,-12);		# Texture's (1,0) is bound to (12,-0.1,-12).
    glTexCoord2d(nrep,nrep);
    glVertex3d( 12,-0.1, 12);		# Texture's (1,1) is bound to (12,-0.1,12).
    glTexCoord2d(0,nrep);
    glVertex3d(-12,-0.1, 12);		# Texture's (0,1) is bound to (-12,-0.1,12).
    glEnd();

    glDisable(GL_TEXTURE_2D);	
    drawFrame(5);				# Draw x, y, and z axis.

cowpos = [] #save cow's position, which will be control points
curInterval = 0 #when animating, it is the index of the point to draw the splines
init = 0 #used when initializing resources necessary to animate
starttime = 0 #save time by glfw.get_time()
M = np.array([[-1,3,-3,1],
            [3,-6,3,0],
            [-3,0,3,0],
            [1,4,1,0]]) #B-spline Matrix
afterPos = []
befdir = [] 
num_rot = 0 #counts number of ratations

def animate(window, x, y):
    global cow2wld_list, cownum, cowpos, init, pickInfo, i, curInterval, starttime, afterPos, befdir, num_rot, doAni, init_cow2wld, isnew,quit
    cow2wld = []
    
    pp=pickInfo

    #Runs only when animating for the first time
    if init == 0:
        starttime = glfw.get_time()
        cow2wld_list.clear()
        cow2wld_list.append(0)
        cownum = 0
        i = 0
        befdir = np.array([1,0,0])
        init = 1

    #counts time
    t = (glfw.get_time() - starttime)

    #when t is over 1, it decreases to 0 in order to maintain number of t in [0,1]
    if t>1:
        t = 0
        starttime = glfw.get_time()
        curInterval = (curInterval + 1) % 6
        #when curInterval reaches 0 again, number of ratation increases
        if curInterval == 0:
            num_rot += 1
    
    #quit animating when number of rotation reaches 3
    if num_rot == 3 or quit==1:
        cow2wld_list[0] = init_cow2wld
        doAni = 0
        quit=0
        init = 0
        num_rot = 0
        isnew = 1
        cowpos.clear()
        return

    
    nextInterval = curInterval
    t2 = t+0.005
    if t2>1:
        t2 -= 1
        nextInterval= (nextInterval + 1) %6
    #calculate position on splines by Marix M
    currentPos =  np.dot(M, np.array([cowpos[(curInterval+5)%6], cowpos[curInterval], cowpos[(curInterval + 1)%6], cowpos[(curInterval+2) %6]]))   
    currentPos = (1/6) * np.dot(np.array([np.power(t,3), np.power(t,2), np.power(t,1), 1]), currentPos)

    #calculate position after small amount of time, in order to get direction that the cow is heading toward.
    afterPos =  np.dot(M, np.array([cowpos[(nextInterval+5)%6], cowpos[nextInterval], cowpos[(nextInterval + 1)%6], cowpos[(nextInterval+2) %6]]))   
    afterPos = (1/6) * np.dot(np.array([np.power(t2,3), np.power(t2,2), np.power(t2,1), 1]), afterPos)

    #calculate direction
    curdir = afterPos -currentPos
    curdir_unit = curdir / np.sqrt(np.dot(curdir, curdir))

    #calculate amount of degree to rotate with y-axis
    curdir_xz = np.array([curdir[0], curdir[2]])
    xz_angle = math.degrees(math.acos(np.dot(curdir_xz, np.array([1,0]))/np.sqrt(np.dot(curdir_xz, curdir_xz))))
    if np.cross(np.array([1,0,0]), np.array([curdir_xz[0], 0, curdir_xz[1]]))[1]<0:
        xz_angle*= -1

    # curdir_xy = np.array([curdir[0], curdir[1]])
    curdir_xz_3d = np.array([curdir[0], 0, curdir[2]])
    pitch_angle = math.degrees(math.acos(np.dot(curdir_xz_3d, curdir_unit)/np.sqrt(np.dot(curdir_xz_3d, curdir_xz_3d))))
    if curdir[1]<0:
        pitch_angle*= -1

    
    glPushMatrix();		        # Push the current matrix of GL into stack.
    glLoadIdentity();		        # Set the GL matrix Identity matrix.bgf
    glTranslated(currentPos[0], currentPos[1], currentPos[2]);	# Set the location of cow.
    
    glRotated(xz_angle, 0,1,0)
    glRotated(pitch_angle, 0,0,1);		# Set the direction of cow.
    
    cow2wld=glGetDoublev(GL_MODELVIEW_MATRIX).T # convert column-major to row-major
    glPopMatrix();			# Pop the matrix on stack to GL.
    
    cow2wld_list[0] = cow2wld
    return

doAni = 0

def display(window):
    global cameraIndex, cow2wld_list, doAni #cow2wld, 
    x, y=glfw.get_cursor_pos(window)
    glClearColor(0.8, 0.9, 0.9, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				# Clear the screen
    # set viewing transformation.
    glLoadMatrixd(wld2cam[cameraIndex].T)

    drawOtherCamera();													# Locate the camera's position, and draw all of them.
    drawFloor();													# Draw floor.
    
    #when setting 6 cows is done, do animating
    if doAni == 1:
        animate(window, x, y)
    
    #draw cows
    i = 0
    for c2w in cow2wld_list:
        drawCow(c2w, cursorOnCowBoundingBox, i)					# Draw cow.
        i += 1

    glFlush();

def reshape(window, w, h):
    width = w;
    height = h;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);            # Select The Projection Matrix
    glLoadIdentity();                       # Reset The Projection Matrix
    # Define perspective projection frustum
    aspect = width/(float)(height);
    gluPerspective(45, aspect, 1, 1024);
    matProjection=glGetDoublev(GL_PROJECTION_MATRIX).T
    glMatrixMode(GL_MODELVIEW);             # Select The Modelview Matrix
    glLoadIdentity();                       # Reset The Projection Matrix

init_cow2wld = []
def initialize(window):
    global cursorOnCowBoundingBox, floorTexID, cameraIndex, camModel, cowModel, init_cow2wld
    cursorOnCowBoundingBox=False;
    # Set up OpenGL state
    glShadeModel(GL_SMOOTH);         # Set Smooth Shading
    glEnable(GL_DEPTH_TEST);         # Enables Depth Testing
    glDepthFunc(GL_LEQUAL);          # The Type Of Depth Test To Do
    # Use perspective correct interpolation if available
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    # Initialize the matrix stacks
    width, height = glfw.get_window_size(window)
    reshape(window, width, height);
    
    # Define lighting for the scene
    lightDirection   = [1.0, 1.0, 1.0, 0];
    ambientIntensity = [0.1, 0.1, 0.1, 1.0];
    lightIntensity   = [0.9, 0.9, 0.9, 1.0];
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientIntensity);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightIntensity);
    glLightfv(GL_LIGHT0, GL_POSITION, lightDirection);
    glEnable(GL_LIGHT0);

    # initialize floor
    im = open('bricks.bmp')
    try:
        ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGB", 0, -1)
    except SystemError:
        ix, iy, image = im.size[0], im.size[1], im.tobytes("raw", "RGBX", 0, -1)

    # Make texture which is accessible through floorTexID. 
    floorTexID=glGenTextures( 1)
    glBindTexture(GL_TEXTURE_2D, floorTexID);		
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, ix, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    # initialize cow
    cowModel=OBJ.OBJrenderer("cow.obj")

    # initialize cow2wld matrix
    glPushMatrix();		        # Push the current matrix of GL into stack.
    glLoadIdentity();		        # Set the GL matrix Identity matrix.
    glTranslated(0,-cowModel.bbmin[1],-8);	# Set the location of cow.
    print(0,-cowModel.bbmin[1],-8)
    glRotated(-90, 0, 1, 0);		# Set the direction of cow. These information are stored in the matrix of GL.
    cow2wld=glGetDoublev(GL_MODELVIEW_MATRIX).T # convert column-major to row-major 
    print(cow2wld)
    init_cow2wld = cow2wld
    glPopMatrix();			# Pop the matrix on stack to GL.

    
    # intialize camera model.
    camModel=OBJ.OBJrenderer("camera.obj")

    # initialize camera frame transforms.

    cameraCount=len(cameras)
    for i in range(cameraCount):
        # 'c' points the coordinate of i-th camera.
        c = cameras[i];										
        glPushMatrix();													# Push the current matrix of GL into stack.
        glLoadIdentity();												# Set the GL matrix Identity matrix.
        gluLookAt(c[0],c[1],c[2], c[3],c[4],c[5], c[6],c[7],c[8]);		# Setting the coordinate of camera.
        wld2cam.append(glGetDoublev(GL_MODELVIEW_MATRIX).T)
        glPopMatrix();													# Transfer the matrix that was pushed the stack to GL.
        cam2wld.append(np.linalg.inv(wld2cam[i]))
    cameraIndex = 0;
   
    cow2wld_list.append(cow2wld)

isnew = 1 #checks if the cow is new or not
x1= -99999
y1 = -99999

def onMouseButton(window,button, state, mods):
    global pickInfo, currentPos, isDrag, V_DRAG, H_DRAG, isnew, doAni, cowpos, x1,y1   #V_DRAG=2 H_DRAG=1 (vertical drag, horizontal drag)
    if doAni==1: return
    GLFW_DOWN=1;
    GLFW_UP=0;
    x, y=glfw.get_cursor_pos(window)

    if button == glfw.MOUSE_BUTTON_LEFT:
        if state == GLFW_DOWN and isDrag != 0:
            x1 = x
            y1 = y
            
            print("no drag->V_mode or click") 
            isDrag=V_DRAG;
            ray=screenCoordToRay(window, x, y)
            planes=[];
            cow=cowModel
            bbmin=cow.bbmin
            bbmax=cow.bbmax

            planes.append(makePlane(bbmin, bbmax, vector3(0,1,0)));
            planes.append(makePlane(bbmin, bbmax, vector3(0,-1,0)));
            planes.append(makePlane(bbmin, bbmax, vector3(1,0,0)));
            planes.append(makePlane(bbmin, bbmax, vector3(-1,0,0)));
            planes.append(makePlane(bbmin, bbmax, vector3(0,0,1)));
            planes.append(makePlane(bbmin, bbmax, vector3(0,0,-1)));

            o=ray.intersectsPlanes(planes);
            cursorOnCowBoundingBox=o[0]
            cowPickPosition=ray.getPoint(o[1])
            cowPickLocalPos=currentPos
            pickInfo=PickInfo( o[1], cowPickPosition, cow2wld_list[cownum], cowPickLocalPos)
           # print( "Left mouse down-click at %d %d\n" % (x,y))
        if state == GLFW_DOWN and isDrag == 0:
            isDrag = V_DRAG
        elif state == GLFW_UP and isDrag!=0:
            if(x1 == x and y1 == y):
                isDrag = 0
                print("setting", len(cow2wld_list),"th cow done")
                cowpos.append(currentPos)

                if len(cow2wld_list) == 6:
                    print("6 cows set, Lets animate!")
                    print("length", len(cowpos))
                    doAni = 1
                    
                isDrag=0 #drag done
                isnew=0
            else:   
                print("V_mode -> H_mode")
            
                ray=screenCoordToRay(window, x, y)

                
                planes=[];
                cow=cowModel
                bbmin=cow.bbmin
                bbmax=cow.bbmax

                planes.append(makePlane(bbmin, bbmax, vector3(0,1,0)));
                planes.append(makePlane(bbmin, bbmax, vector3(0,-1,0)));
                planes.append(makePlane(bbmin, bbmax, vector3(1,0,0)));
                planes.append(makePlane(bbmin, bbmax, vector3(-1,0,0)));
                planes.append(makePlane(bbmin, bbmax, vector3(0,0,1)));
                planes.append(makePlane(bbmin, bbmax, vector3(0,0,-1)));

                o=ray.intersectsPlanes(planes);
                cursorOnCowBoundingBox=o[0]
                cowPickPosition=ray.getPoint(o[1])
                cowPickLocalPos=currentPos
                pickInfo=PickInfo( o[1], cowPickPosition, cow2wld_list[cownum], cowPickLocalPos)
            
            
                isDrag=H_DRAG;
                print( "Left mouse up\n");
                # start horizontal dragging using mouse-move events.
        elif state == GLFW_UP and isDrag==0:
            isDrag = H_DRAG
    elif button == glfw.MOUSE_BUTTON_RIGHT:
        if state == GLFW_DOWN:
            print( "Right mouse click at (%d, %d)\n"%(x,y) );

def onMouseDrag(window, x, y):
    global currentPos, isDrag,cursorOnCowBoundingBox, pickInfo, cow2wld_list, cownum, isnew # cow2wld
    if doAni==1: return #if animating is happening, do nothing
    if isDrag: 
        print( "in drag mode %d\n"% isDrag);
        if isDrag==V_DRAG and cursorOnCowBoundingBox: 
            ray=screenCoordToRay(window, x, y);
            pp=pickInfo;
            
            p=Plane(np.array((0,0,1)), pp.cowPickPosition); 
            c=ray.intersectsPlane(p); # gets t which that ray intersect plane
            tempPos1=ray.getPoint(c[1])
            
            p2=Plane(np.array((1,0,0)), pp.cowPickPosition); 
            c2=ray.intersectsPlane(p2); # gets t which that ray intersect plane
            tempPos2=ray.getPoint(c2[1])

            currentPos = [tempPos2[0], tempPos1[1], tempPos1[2]]
            
            print("cowpickpos: ", pp.cowPickPosition," currpos: ", currentPos)
            #print(pp.cowPickConfiguration, cow2wld)
            T=np.eye(4)
            setTranslation(T, currentPos-pp.cowPickPosition)
            cow2wld=T@pp.cowPickConfiguration
            if isnew == 0:
                cow2wld_list.append(cow2wld)
                cownum += 1
                isnew = 1
            elif isnew == 1:
                cow2wld_list[cownum] = cow2wld
            # vertical dragging
            # TODO:
            # create a dragging plane perpendicular to the ray direction, 
            # and test intersection with the screen ray.
            print('vdrag')

        else:
            # horizontal dragging
            # Hint: read carefully the following block to implement vertical dragging.
            if cursorOnCowBoundingBox:
                ray=screenCoordToRay(window, x, y);
                pp=pickInfo;
                p=Plane(np.array((0,1,0)), pp.cowPickPosition); # plane which is parallel to y=0 and height=cowpickPoisition->cursor
                c=ray.intersectsPlane(p); # gets t which that ray intersect plane
                
                currentPos=ray.getPoint(c[1])
                print("cowpickpos: ", pp.cowPickPosition," currpos: ", currentPos)
                #print(pp.cowPickConfiguration, cow2wld)
                T=np.eye(4)
                setTranslation(T, currentPos-pp.cowPickPosition)
                cow2wld=T@pp.cowPickConfiguration
                currentPos = getTranslation(cow2wld)
                if isnew == 1:
                    cow2wld_list[cownum] = cow2wld
                else:
                    cow2wld_list.append(cow2wld) 
                    cownum += 1
                    isnew = 1
    else:
        ray=screenCoordToRay(window, x, y)

        planes=[];
        cow=cowModel
        bbmin=cow.bbmin
        bbmax=cow.bbmax

        planes.append(makePlane(bbmin, bbmax, vector3(0,1,0)));
        planes.append(makePlane(bbmin, bbmax, vector3(0,-1,0)));
        planes.append(makePlane(bbmin, bbmax, vector3(1,0,0)));
        planes.append(makePlane(bbmin, bbmax, vector3(-1,0,0)));
        planes.append(makePlane(bbmin, bbmax, vector3(0,0,1)));
        planes.append(makePlane(bbmin, bbmax, vector3(0,0,-1)));

        o=ray.intersectsPlanes(planes);
        cursorOnCowBoundingBox=o[0]
        cowPickPosition=ray.getPoint(o[1])
        cowPickLocalPos=transform(np.linalg.inv(cow2wld_list[cownum]),cowPickPosition)
        pickInfo=PickInfo( o[1], cowPickPosition, cow2wld_list[cownum], cowPickLocalPos)
        #__init__(self, cursorRayT, cowPickPosition, cowPickConfiguration, cowPickPositionLocal)
      

def screenCoordToRay(window, x, y):
    width, height = glfw.get_window_size(window)

    matProjection=glGetDoublev(GL_PROJECTION_MATRIX).T
    matProjection=matProjection@wld2cam[cameraIndex]; # use @ for matrix mult.
    invMatProjection=np.linalg.inv(matProjection);
    # -1<=v.x<1 when 0<=x<width
    # -1<=v.y<1 when 0<=y<height
    vecAfterProjection =vector4(
            (float(x - 0))/(float(width))*2.0-1.0,
            -1*(((float(y - 0))/float(height))*2.0-1.0),
            -10)

    #std::cout<<"cowPosition in clip coordinate (NDC)"<<matProjection*cow2wld.getTranslation()<<std::endl;
	
    vecBeforeProjection=position3(invMatProjection@vecAfterProjection);

    rayOrigin=getTranslation(cam2wld[cameraIndex])
    return Ray(rayOrigin, normalize(vecBeforeProjection-rayOrigin))

def main():
    global doAni

    if not glfw.init():
        print ('GLFW initialization failed')
        sys.exit(-1)
    width = 800;
    height = 600;
    window = glfw.create_window(width, height, '2017029616_PA2', None, None)
    if not window:
        glfw.terminate()
        sys.exit(-1)

    glfw.make_context_current(window)
    glfw.set_key_callback(window, onKeyPress)
    glfw.set_mouse_button_callback(window, onMouseButton)
    glfw.set_cursor_pos_callback(window, onMouseDrag)
    glfw.swap_interval(1)

    initialize(window);						
    while not glfw.window_should_close(window):
        
        glfw.poll_events()
        display(window)

        glfw.swap_buffers(window)

    glfw.terminate()
if __name__ == "__main__":
    main()
