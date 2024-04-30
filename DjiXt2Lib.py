#!/usr/bin/env python3
"""DJIXT2Lib.py: Python librairy for the DJI XT2 Thermal Sensor."""


import os
import argparse
import numpy as np
import cameratransform as ct
from pyproj import Transformer
import warnings
from exiftool import ExifToolHelper
warnings.filterwarnings("ignore", category=DeprecationWarning)
from dataclasses import dataclass
from libtiff import TIFFimage
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
    plot : bool = False
    def __post_init__(self):
        self.name,ext = os.path.splitext(self.FileName)

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

        print("Calcul spaceFromImage")
        self.C1=self.camera.spaceFromImage([0, 0],Z=0)
        self.C2=self.camera.spaceFromImage([self.image_width, 0],Z=0)
        self.C3=self.camera.spaceFromImage([0, self.image_height],Z=0)
        self.C4=self.camera.spaceFromImage([self.image_width ,self.image_height],Z=0)
        self.xmin=min(self.C1[0],self.C2[0],self.C3[0],self.C4[0])
        self.xmax=max(self.C1[0],self.C2[0],self.C3[0],self.C4[0])
        self.ymin=min(self.C1[1],self.C2[1],self.C3[1],self.C4[1])
        self.ymax=max(self.C1[1],self.C2[1],self.C3[1],self.C4[1])
        self.footprint_box = [self.xmin,self.xmax,self.ymin,self.ymax]

    def compute_emission(self):
        emission_matrix = np.empty([self.image_height, self.image_width])
        for i in range(self.image_width):
            offset_line, ray_line_matrix = self.camera.getRay([[i, j] for j in range(self.image_height)],normed=True)
            theta_matrix = np.arctan2(np.sqrt(ray_line_matrix[:,0]**2+ray_line_matrix[:,1]**2),-ray_line_matrix[:,2])*180.0/np.pi
            emission_matrix[:,i]=theta_matrix
        emission = self.camera.getTopViewOfImage(emission_matrix, self.footprint_box, scaling=self.gsd, do_plot=False)
        if self.plot:
           self.plot('Emission',emission)


        self. write_worldfile_geotiff(emission,"emission","emission")

    def plot_image(self,title, image):
        cv2.imshow(title, image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def write_worldfile_geotiff(self,image:np.ndarray, description:str, suffix:str):
         # Word file ortho
        WorldFile = (self.gsd,0,0, -self.gsd, self.xmin, self.ymin)

        wrdFile = f'{self.name}_{suffix}.tfw'
        with open(wrdFile, 'w') as f_handle:
            np.savetxt(f_handle, WorldFile, fmt="%.3f")

        # Write orthophoto
        myGeoTIFF = f'{self.name}_{suffix}.tif'

        tiff = TIFFimage(image, description=f"{self.FileName} {description}")
        tiff.write_file(myGeoTIFF, compression=None) # or 'lzw'



    def compute_ortho(self,gsd=0.1):

        print("Calcul ortho")

        im_array = io.imread(self.FileName)
        #__import__("IPython").embed()

        ortho = self.camera.getTopViewOfImage(im_array*self.TGain, self.footprint_box, scaling=self.gsd, do_plot=False)

        if self.plot:
            self.plot("Orthophoto", ortho)

        self.write_worldfile_geotiff(ortho,"ortho", "orthophoto")


def main() -> None:
    """
    Create GEOtiff from XT2 thermal Tiff
    """
    parser = argparse.ArgumentParser(description="Create GEOtiff from XT2 thermal Tiff.")
    parser.add_argument("input_file", help="Thermal input tiff file")

    parser.add_argument("output_epsg", type=int,help="EPSG code for output GEOtiff. Ex: 4471")

    parser.add_argument("--rtk_ground_reference", type=float,
                        help="sensor width in mm.",default=NIVEAU_LAC,
                        required=False)
    parser.add_argument("--sensor_width", type=float,help="sensor width in mm.",default=8.7,required=False)
    parser.add_argument("--sensor_height", type=float,help="sensor height in mm.",default=6.22,required=False)
    parser.add_argument("--gain", type=int,help="numerical gain",default=150,required=False)
    parser.add_argument("-emission",help="Ccompute emission",action='store_true')
    parser.add_argument("-no_ortho",help="Do not compute orthophoto",action='store_true')
    parser.add_argument("-plot",help="Plot images",action='store_true')

    args = parser.parse_args()



    inProj = 'EPSG:4326' # WGS-84 [dd.mm.sssd]
    outProj = f'EPSG:{args.output_epsg}'


    # Transformation des coordonnées
    transformer = Transformer.from_crs(inProj,outProj)
    image_test=imageclass(args.input_file,(args.sensor_width, args.sensor_height),gain=args.gain, plot=args.plot)

    image_test.local_longitude,image_test.local_latitude  = transformer.transform(image_test.latitude,image_test.longitude)


    # On utilise l'altitude RTK du drone et du lac pour trouver la hauteur au lac'
    image_test.relative_altitude = image_test.absolute_altitude - args.rtk_ground_reference
    print(f'Drone RTK altitude{image_test.absolute_altitude} Drone to ground distance {image_test.relative_altitude}')
    print("Calcul camera")
    image_test.compute_camera()
    if not args.no_ortho :
        image_test.compute_ortho() # orthoimage computation from camera and sensor model
    if args.emission :
            image_test.compute_emission()



if __name__ == "__main__":
    main()
