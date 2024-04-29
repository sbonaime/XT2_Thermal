#!/usr/bin/env python3
"""DJIXT2Lib.py: Python librairy for the DJI XT2 Thermal Sensor."""


import argparse
import sys
import numpy as np
import cameratransform as ct
import os
from pyproj import Proj,Transformer
import warnings
from exiftool import ExifToolHelper,ExifTool
warnings.filterwarnings("ignore", category=DeprecationWarning) 
from dataclasses import dataclass
from icecream import ic
from libtiff import TIFF,TIFFimage
import numpy as np
import cv2
from skimage import io

EXIFTOOL_PATH = '/usr/local/bin/exiftool'
NIVEAU_LAC=-17.832377624511718-1.8011


@dataclass 
class imageclass():
    FileName : str
    sensor_size : tuple # in mm. To check for DJI XT2 thermal. Not in exif..
    local_longitude : float = 0
    local_latitude : float = 0
    gain : int = 0
    def __post_init__(self):
        self.sensor_width,self.sensor_heigh = self.sensor_size 
        with ExifToolHelper(executable=EXIFTOOL_PATH)as et:
            self.metadata=et.get_metadata(self.FileName)[0]  
            #print(self.metadata)
        if 'EXIF:ImageWidth' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.image_width=int(self.metadata['EXIF:ImageWidth'])
        if 'EXIF:ImageHeight' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.image_height=int(self.metadata['EXIF:ImageHeight'])

        if 'EXIF:FocalLength' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.focal_length=float(self.metadata['EXIF:FocalLength'])

        if 'Composite:FocalLength35efl' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.focal_length_35_mm=float(self.metadata['Composite:FocalLength35efl'])

        if 'XMP:GimbalRollDegree' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.roll=float(self.metadata['XMP:GimbalRollDegree'])

        if 'XMP:GimbalYawDegree' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.yaw=float(self.metadata['XMP:GimbalYawDegree'])
            
        if 'XMP:GimbalPitchDegree' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            #### +90 car avec Drone DJI NADIR = -90####
            self.pitch=float(self.metadata['XMP:GimbalPitchDegree'])+90

        if 'Composite:GPSLatitude' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.latitude=float(self.metadata['Composite:GPSLatitude'])

        if 'Composite:GPSLongitude' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.longitude=float(self.metadata['Composite:GPSLongitude'])

        if 'XMP:AbsoluteAltitude' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.absolute_altitude=float(self.metadata['XMP:AbsoluteAltitude'])

        if 'XMP:RelativeAltitude' in self.metadata:
            self.relative_altitude=float(self.metadata['XMP:RelativeAltitude'])

        if 'XMP:TlinearGain' in self.metadata:
            #print(self.metadata['EXIF:ImageWidth'])
            self.TGain=float(self.metadata['XMP:TlinearGain'])
            if self.gain != 0:
                self.TGain = self.gain
        
        self.gsd = (self.sensor_width * self.relative_altitude) / (self.focal_length * self.image_width)
       

    def compute_camera(self):
    # initialize the camera    
        self.camera = ct.Camera(ct.RectilinearProjection(focallength_mm=self.focal_length,
                                            sensor=self.sensor_size,
                                            image=(self.image_width,self.image_height)),
                ct.SpatialOrientation(elevation_m=self.relative_altitude,
                                        tilt_deg=self.pitch,
                                        heading_deg=self.yaw,
                                        roll_deg=self.roll,
                                        pos_x_m=self.local_longitude,
                                        pos_y_m=self.local_latitude))

def camToOrtho(image:imageclass,gds=0.1):
    


    print("Calcul spaceFromImage")

    C1=image.camera.spaceFromImage([0, 0],Z=0)
    C2=image.camera.spaceFromImage([image.image_width, 0],Z=0)
    C3=image.camera.spaceFromImage([0, image.image_height],Z=0)
    C4=image.camera.spaceFromImage([image.image_width ,image.image_height],Z=0)



#    __import__("IPython").embed()
    # print("Calcul emissions")
    # emission = np.empty([image.image_height, image.image_width])
    
    # for i in range(image.image_width):
    #     for j in range(image.image_height):
    #         offset, ray = image.camera.getRay([i, j],normed=True)
    #         p = ray[0]/ray[2]
    #         pt = ray[1]/ray[2]
    #         theta = np.arctan2(np.sqrt(ray[0]**2+ray[1]**2),-ray[2])*180.0/np.pi
    #         emission[j][i] = theta

    # emission_matrix = np.empty([image.image_height, image.image_width])

    # for i in range(image.image_width):
            
    #         offset_line, ray_line_matrix = image.camera.getRay([[i, j] for j in range(image.image_height)],normed=True)
            
    #         p_matrix = ray_line_matrix[:,0]/ray_line_matrix[:,2]
    #         pt_matrix = ray_line_matrix[:,1]/ray_line_matrix[:,2]
            
    #         theta_matrix = np.arctan2(np.sqrt(ray_line_matrix[:,0]**2+ray_line_matrix[:,1]**2),-ray_line_matrix[:,2])*180.0/np.pi
    #         #theta = np.arctan2(np.sqrt(ray[0]**2+ray[1]**2),-ray[2])*180.0/np.pi
            
    #         #emission[j][i] = theta
    #         np.append(emission_matrix,theta_matrix)


    xmin=min(C1[0],C2[0],C3[0],C4[0])
    xmax=max(C1[0],C2[0],C3[0],C4[0])
    ymin=min(C1[1],C2[1],C3[1],C4[1])
    ymax=max(C1[1],C2[1],C3[1],C4[1])


    print("Calcul ortho")

    im_array = io.imread(image.FileName)
    #__import__("IPython").embed()

    ortho = image.camera.getTopViewOfImage(im_array*image.TGain, [xmin,xmax,ymin,ymax], scaling=image.gsd, do_plot=False)
   # emission_o = image.camera.getTopViewOfImage(emission, [xmin,xmax,ymin,ymax], scaling=image.gsd, do_plot=False)

    #cv2.imshow('tiff', im_array*10)
    #cv2.waitKey(0)

    #cv2.imshow('ortho', ortho)
    #cv2.waitKey(0)
    #cv2.imshow('emission_o', emission_o)
   # cv2.waitKey(0) 

    # closing all open windows 
    cv2.destroyAllWindows() 




    xres = image.gsd
    yres = image.gsd

    # geotransform = (xmin, xres, 0, ymax, 0, -yres)

    # Word file ortho
    WorldFile = (xres,0,0, -yres, xmin, ymin)
    Name,ext = os.path.splitext(image.FileName)


    wrdFile = f'{Name}_2_ortho.tfw'
    with open(wrdFile, 'w') as f_handle:
        np.savetxt(f_handle, WorldFile, fmt="%.3f")

    # Ortho
    myGeoTIFF_2 = f'{Name}_2_ortho.tif'
    

    tiff = TIFFimage(ortho, description='image.FileName')
    tiff.write_file(myGeoTIFF_2, compression=None) # or 'lzw'

    # # Word file emission
    # WorldFile = (xres,0,0, -yres, xmin, ymax)
    # Name,ext = os.path.splitext(image.FileName)
    # wrdFile = f'{Name}_emission.tfw'
    # myGeoTIFF = f'{Name}_emission.tif'
    # with open(wrdFile, 'w') as f_handle:
    #     np.savetxt(f_handle, WorldFile, fmt="%.3f")
    # tif = TIFF.open(myGeoTIFF, mode='w')
    # tif.write_image(emission_o)




#Sensor['Size'] = (8.7, 6.22)   # in mm (this field is not present in the DJI XT2 Exif) ?? check for the Thermal sensor



def main() -> None:
    """
    Create GEOtiff from XT2 thermal Tiff
    """
    parser = argparse.ArgumentParser(description="Create GEOtiff from XT2 thermal Tiff.")
    parser.add_argument("input_file", help="Thermal input tiff file")
                        
    parser.add_argument("output_epsg", type=int,
                        help="EPSG code for output GEOtiff. Ex: 4471")
    
    parser.add_argument("--rtk_ground_reference", type=float,
                        help="sensor width in mm.",default=NIVEAU_LAC,
                        required=False)
    parser.add_argument("--sensor_width", type=float,
                        help="sensor width in mm.",default=8.7,
                        required=False)
    parser.add_argument("--sensor_height", type=float,
                        help="sensor height in mm.",default=6.22,
                        required=False)
    parser.add_argument("--gain", type=int,
                        help="numerical gain",default=150,
                        required=False)
    
    args = parser.parse_args()



    inProj = 'EPSG:4326' # WGS-84 [dd.mm.sssd]
    outProj = f'EPSG:{args.output_epsg}'


    # Transformation des coordonnées
    transformer = Transformer.from_crs(inProj,outProj)
    image_test=imageclass(args.input_file,(args.sensor_width, args.sensor_height),gain=args.gain)

    image_test.local_longitude,image_test.local_latitude  = transformer.transform(image_test.latitude,image_test.longitude)


    # On utilise l'altitude RTK du drone et du lac pour trouver la hauteur au lac'

    image_test.relative_altitude = image_test.absolute_altitude - args.rtk_ground_reference
    print(f'Drone RTK altitude{image_test.absolute_altitude} Drone to ground distance {image_test.relative_altitude}')
    print("Calcul camera")
    image_test.compute_camera()
    #ic(image_test)
    camToOrtho(image_test); # orthoimage computation from camera and sensor model



if __name__ == "__main__":
    main()
