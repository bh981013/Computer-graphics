# !/usr/bin/env python3
# -*- coding: utf-8 -*
# sample_python aims to allow seamless integration with lua.
# see examples below

import os
import sys
import pdb  # use pdb.set_trace() for debugging
import code # or use code.interact(local=dict(globals(), **locals()))  for debugging.
import xml.etree.ElementTree as ET
import numpy as np
import warnings
from PIL import Image

class shader:
    name = ''
    diffuse = []
    spec = []
    exp = 0
    def __init__(self, name, diffuse, spec, exp):
        self.name = name
        self.diffuse = diffuse
        self.spec = spec
        self.exp = exp

class Sphere:
    def __init__(self, point, radius):
        self.Point = np.array(point)
        self.Radius = radius

class Color:
    def __init__(self, R, G, B):
        self.color=np.array([R,G,B]).astype(np.float)

    # Gamma corrects this color.
    # @param gamma the gamma value to use (2.2 is generally used).
    def gammaCorrect(self, gamma):
        inverseGamma = 1.0 / gamma;
        self.color=np.power(self.color, inverseGamma)

    def toUINT8(self):
        return (np.clip(self.color, 0,1)*255).astype(np.uint8)


def main():
    tree = ET.parse(sys.argv[1])
    root = tree.getroot()
    print(tree)
    print(root)
    # set default values
    viewPoint=np.array([0,0,0]).astype(np.float)
    viewDir=np.array([0,0,-1]).astype(np.float)
    viewUp=np.array([0,1,0]).astype(np.float)
    viewProjNormal=-1*viewDir  # you can safely assume this. (no examples will use shifted perspective camera)
   
    projDistance=1.0
    
    viewWidth=1.0
    viewHeight=1.0

    diffuseColor_c=np.array([0,0,0]).astype(np.float)
    specularColor_c=np.array([0,0,0]).astype(np.float)
    isSpec = False

    #color
    SdcolorArray = []
    SscolorArray = []
    BdcolorArray = []
    BscolorArray = []
    exponent_c = 1.0

    shaders = {}
    #Sphere
    center = np.array([0,0,0]).astype(np.float)
    centerArray = []
    radius = 1.0
    radiusArray = []
    
    #Box
    minPt = np.array([0,0,0])
    minPtArr = []
    maxPt = np.array([0,0,0])
    maxPtArr = []

    #light
    l_intensity=np.array([1,1,1]).astype(np.float)  # how bright the light is.
    l_intArr = []
    l_position = np.array([0,0,0]).astype(np.float)
    l_posArr = []
    
    
    print(np.cross(viewDir, viewUp))

    imgSize=np.array(root.findtext('image').split()).astype(np.int)
    
    for c in root.findall('camera'):
        viewPoint=np.array(c.findtext('viewPoint').split()).astype(np.float)
        viewDir = np.array(c.findtext('viewDir').split()).astype(np.float)
        viewUp = np.array(c.findtext('viewUp').split()).astype(np.float)
        if c.findtext('projDistance'):
            projDistance = np.array(c.findtext('projDistance').split()).astype(np.float)
        viewWidth = np.array(c.findtext('viewWidth').split()).astype(np.float)
        viewHeight = np.array(c.findtext('viewHeight').split()).astype(np.float)
        print('viewpoint', viewPoint)
        print('viewdir', viewDir)
        print('viewUp', viewUp)
        
    for c in root.findall('shader'):
        if c.findtext('diffuseColor'):
            diffuseColor_c=np.array(c.findtext('diffuseColor').split()).astype(np.float)
            #dcolorArray.append(diffuseColor_c)
        else:
            diffuseColor_c = (np.array([0,0,0]))

        if c.findtext('specularColor'):
            specularColor_c=np.array(c.findtext('specularColor').split()).astype(np.float)
            #scolorArray.append(specularColor_c)
        else:
            specularColor_c = (np.array([0,0,0]))

        if c.findtext('exponent'):
            exponent_c=np.array(c.findtext('exponent').split()).astype(np.float)
        
        shaders[c.attrib["name"]] = shader(c.attrib["name"], diffuseColor_c, specularColor_c, exponent_c)
        
        #print('name', c.get('name'))
        print('diffuseColor', diffuseColor_c)

    i = 0    
    for c in root.findall('surface'):
        if c.attrib['type'] == 'Sphere':
            radius = np.array(c.findtext('radius').split()).astype(np.float) 
            radiusArray.append(radius)
            center = np.array(c.findtext('center').split()).astype(np.float)
            centerArray.append(center)

        if c.attrib['type'] == 'Box':
            minPt = np.array(c.findtext('minPt').split()).astype(np.float)
            minPtArr.append(minPt)
            maxPt = np.array(c.findtext('maxPt').split()).astype(np.float)
            maxPtArr.append(maxPt)

        
        if c.attrib["type"] == "Sphere":
            SdcolorArray.append(shaders[c.find("shader").attrib["ref"]].diffuse)
            SscolorArray.append(shaders[c.find("shader").attrib["ref"]].spec)
        elif c.attrib["type"] == "Box":
            BdcolorArray.append(shaders[c.find("shader").attrib["ref"]].diffuse)
            BscolorArray.append(shaders[c.find("shader").attrib["ref"]].spec)

    print('minPtArr:', minPtArr)
    print('maxptarr:', maxPtArr)

    for c in root.findall('light'):
        l_position = np.array(c.findtext('position').split()).astype(np.float)
        l_posArr.append(l_position)
        l_intensity = np.array(c.findtext('intensity').split()).astype(np.float)
        l_intArr.append(l_intensity)
        print("l_posArr", l_posArr)
        print("l_intArr", l_intArr)
    #code.interact(local=dict(globals(), **locals()))  

    # Create an empty image
    channels=3
    img = np.zeros((imgSize[1], imgSize[0], channels), dtype=np.uint8)
    print("imgsize1: " , imgSize[1]) #세로, 
    print("imgsize0: " , imgSize[0]) #가로
    img[:,:]=0
    # replace the code block below!
    print('BdcolorArray', BdcolorArray)
    print('BscolorArray', BscolorArray)
    print('centerArr', centerArray)
    print('radiiusarr', radiusArray)
    w = viewDir
    w = w / np.sqrt(np.dot(w, w))
    
    u = np.cross(w, viewUp)
    u = u / np.sqrt(np.dot(u, u))
    v = np.cross(w, u)
    print("exponent", exponent_c)
    
    
    for i in np.arange(imgSize[0]):
        for j in np.arange(imgSize[1]):
            #convert pixel to image
            l = (-viewWidth) /2
            b = (-viewHeight) / 2
          
            x = l + (viewWidth * (i + 0.5) / imgSize[0])
            y = b + (viewHeight * (j + 0.5) / imgSize[1])

            p = viewPoint
            s = p + x * u + y * v + projDistance * w
            d = s - p #ray vector
            d = d / np.sqrt(np.dot(d,d))
            
            Dcolor = np.array([0,0,0])
            Scolor = np.array([0,0,0])

            isSphere = 0;
            #if isSphere == 1
            Center = np.array([0,0,0])
            Radius = 0
            
            #if isSphere == 0
            minSlab = 0
            maxSlab = 0
            MinPt = np.array([0,0,0])
            MaxPt = np.array([0,0,0])

            isShadow = 0
            lightVec = []
            #check if the ray hits object
            t = sys.maxsize;

            for dcolor, scolor, center, radius in zip(SdcolorArray, SscolorArray, centerArray, radiusArray):
                leng = p - center
                if ((np.dot(leng,d)**2) - np.dot(d, d)* (np.dot(leng, leng) - radius ** 2)) >= 0 : 
                    temp = (-1) * np.dot(leng,d) - np.sqrt((np.dot(leng,d)**2) - (np.dot(leng, leng) - radius ** 2))
                    if temp < t:
                        Dcolor = dcolor
                        Scolor = scolor
                        Center = center
                        Radius = radius
                        isSphere = 1
                        t = temp
            for dcolor, scolor, minPt, maxPt in zip(BdcolorArray, BscolorArray, minPtArr, maxPtArr):
                x0 = (minPt[0] - p[0]) / d[0]
                y0 = (minPt[1] - p[1]) / d[1]
                z0 = (minPt[2] - p[2]) / d[2]

                x1 = (maxPt[0] - p[0])/ d[0]
                y1 = (maxPt[1] - p[1]) / d[1]
                z1 = (maxPt[2] - p[2]) / d[2]

                txmax = max(x0,x1)
                tymax = max(y0,y1)
                tzmax = max(z0,z1)

                txmin = min(x0,x1)
                tymin = min(y0,y1)
                tzmin = min(z0,z1)

                tmin = max(txmin, tymin, tzmin)
                tmax = min(txmax, tymax, tzmax)
    
                if tmin > tmax: continue

                

                if tmin < t:
                    if txmin == tmin: minSlab = 0
                    elif tymin == tmin: minSlab = 1
                    elif tzmin == tmin: minSlab = 2
                    
                    if txmax == tmax: maxSlab = 0
                    elif tymax == tmax: maxSlab = 1
                    elif tzmax == tmax: maxSlab = 2

                    Dcolor = dcolor
                    Scolor = scolor
                    MinPt = minPt
                    MaxPt = maxPt
                    isSphere = 0
                    t = tmin

            if t == sys.maxsize: continue
            
            L_color = Color(0.,0.,0.)
            #D-shading
            
            intersect = p + d * t
    
            n = np.array([0,0,0])
            if isSphere:
                n = intersect - Center
                n = n/np.sqrt(np.dot(n,n))
            else:   
                if d[minSlab] > 0:
                    n[minSlab] = -1
                else: n[minSlab] = 1                

            L_red = 0
            L_green = 0
            L_blue = 0

            for l_position, l_intensity in zip(l_posArr, l_intArr):
                temp_int = l_intensity.copy()
                light = (l_position) - intersect
                light = light / np.sqrt(np.dot(light, light))
                intersect_sh = intersect + light * sys.float_info.min
                

            
                for center, radius in zip(centerArray, radiusArray):
                    if all((center-Center) == [0,0,0]) and all((radius-Radius) == [0.,0,0]):
                        continue
                    check = ((np.dot(light, intersect_sh-center)**2) - (np.dot(light, light)*(np.dot(intersect_sh - center, intersect_sh - center) - radius ** 2)))
                    if check >= 0:
                        if (-np.dot(light, intersect_sh - center) - check) > 0:
                            temp_int = [0,0,0]
                            break
            
                for minPt, maxPt in zip(minPtArr, maxPtArr):
                    if all((minPt - MinPt) == [0,0,0]):
                        continue
                    x0 = (minPt[0] - intersect_sh[0])/ light[0]
                    x1 = (maxPt[0] - intersect_sh[0])/ light[0]
                    y0 = (minPt[1] - intersect_sh[1]) / light[1]
                    y1 = (maxPt[1] - intersect_sh[1]) / light[1]
                    z0 = (minPt[2] - intersect_sh[2]) / light[2]
                    z1 = (maxPt[2] - intersect_sh[2]) / light[2]
                    
                    txmax = max(x0,x1)
                    tymax = max(y0,y1)
                    tzmax = max(z0,z1)
                    
                    txmin = min(x0,x1)
                    tymin = min(y0,y1)
                    tzmin = min(z0,z1)

                    tmin = max(txmin, tymin, tzmin)
                    tmax = min(txmax, tymax, tzmax)

                    if tmin < tmax and tmin> 0 and tmax > 0:
                        temp_int = [0,0,0]
                        break
                
                #S-shading
                
                view = p - intersect
                view = view / np.sqrt(np.dot(view, view))
                h = view + light
                h = h / np.sqrt(np.dot(h,h))
                L_red += float(Dcolor[0] * temp_int[0] * max(0, np.dot(n, light)) + Scolor[0] * temp_int[0] * ((max(0, np.dot(n, h)))**exponent_c))
                L_green += Dcolor[1] * temp_int[1] * max(0, np.dot(n, light)) + Scolor[1] * temp_int[1] * ((max(0, np.dot(n, h)))**exponent_c)
                L_blue += Dcolor[2] * temp_int[2] * max(0, np.dot(n, light)) + Scolor[2] * temp_int[2] * ((max(0, np.dot(n, h)))**exponent_c)
                    
                
            L_color = Color(L_red, L_green, L_blue)
            L_color.gammaCorrect(2.2)         
            img[j][i] = L_color.toUINT8()

                
                                            
               



            
             
    rawimg = Image.fromarray(img, 'RGB')
    #rawimg.save('out.png')
    rawimg.save(sys.argv[1]+'.png')
    file = sys.argv[1] + '.png'
    img = Image.open(file)
    img.show()


if __name__=="__main__":
    warnings.filterwarnings('ignore')
    main()
    
