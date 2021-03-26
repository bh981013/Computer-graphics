import glfw
from OpenGL.GL import *
import numpy as np

global type
type = GL_LINE_LOOP

def render():
    glClear(GL_COLOR_BUFFER_BIT)

    glBegin(type)
    arr = np.linspace(0,np.radians(360),12, False, float)
    for i in range(12):
        glVertex2f(np.cos(arr[0][i]), np.sin(arr[0][i]))
    glEnd()
    
def key_callback(window, key, scancode, action, mods):
    global type
    if key ==glfw.KEY_1 and action == glfw.PRESS:
        type = GL_POINTS
    if key == glfw.KEY_2 and action == glfw.PRESS:
        type = GL_LINES
    if key == glfw.KEY_3 and action == glfw.PRESS:
        type = GL_LINE_STRIP
    if key == glfw.KEY_4 and action == glfw.PRESS:
        type = GL_LINE_LOOP
    if key == glfw.KEY_5 and action == glfw.PRESS:
        type = GL_TRIANGLES
    if key == glfw.KEY_6 and action == glfw.PRESS:
        type = GL_TRIANGLE_STRIP
    if key == glfw.KEY_7 and action == glfw.PRESS:
        type = GL_TRIANGLE_FAN
    if key == glfw.KEY_8 and action == glfw.PRESS:
        type = GL_QUADS
    if key == glfw.KEY_9 and action == glfw.PRESS:
        type = GL_QUAD_STRIP
    if key == glfw.KEY_0 and action == glfw.PRESS:
        type = GL_POLYGON
    
def main():
    if not glfw.init():
        return
    window = glfw.create_window(480, 480, "2017029616-3-1", None, None)
    if not window:
        glfw.terminate()
        return
    glfw.set_key_callback(window, key_callback)
    glfw.make_context_current(window)
    
    while not glfw.window_should_close(window):
        glfw.poll_events()
        render()
        glfw.swap_buffers(window)
    glfw.terminate()
    
if __name__ == "__main__":
    main()
