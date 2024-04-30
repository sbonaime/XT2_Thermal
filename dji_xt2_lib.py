#!/usr/bin/env python3
"""DJIXT2Lib.py: Python library for the DJI XT2 Thermal Sensor."""
__author__ = "Antoine LUCAS, Sebastien BONAIME"
__status__ = "Dev"
__version__ = "1.1.0"


import os,sys
import argparse
from dataclasses import dataclass, field
from libtiff import TIFFimage
import numpy as np
import cameratransform as ct
from pyproj import Transformer
from exiftool import ExifToolHelper
import cv2
from skimage import io

EXIFTOOL_PATH = '/usr/local/bin/exiftool'
NIVEAU_LAC=-17.832377624511718-1.8011


@dataclass
class ImageClass():
    file_name : str
    sensor_size : tuple # in mm. To check for DJI XT2 thermal. Not in exif..
    local_longitude : float = 0
    local_latitude : float = 0
    gain : int = 0
    plot : bool = False
    corner_1 : int = 0
    corner_2 : int = 0
    corner_3 : int = 0
    corner_4 : int = 0
    footprint_x_min : int = 0
    footprint_x_max : int = 0
    footprint_y_min : int = 0
    footprint_y_max : int = 0
    footprint_box : list[int] = field(default_factory=list)

    def __post_init__(self):
        self.name,_ = os.path.splitext(self.file_name)

        self.sensor_width,self.sensor_heigh = self.sensor_size
        with ExifToolHelper(executable=EXIFTOOL_PATH)as et:
            self.metadata=et.get_metadata(self.file_name)[0]
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
            self.thermal_gain=float(self.metadata['XMP:TlinearGain'])
            if self.gain != 0:
                self.thermal_gain = self.gain
        self.gsd = (self.sensor_width * self.relative_altitude)\
              / (self.focal_length * self.image_width)

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
        self.corner_1=self.camera.spaceFromImage([0, 0],Z=0)
        self.corner_2=self.camera.spaceFromImage([self.image_width, 0],Z=0)
        self.corner_3=self.camera.spaceFromImage([0, self.image_height],Z=0)
        self.corner_4=self.camera.spaceFromImage([self.image_width ,self.image_height],Z=0)


        self.footprint_x_min=min(self.corner_1[0],self.corner_2[0],self.corner_3[0],self.corner_4[0])
        self.footprint_x_max=max(self.corner_1[0],self.corner_2[0],self.corner_3[0],self.corner_4[0])
        self.footprint_y_min=min(self.corner_1[1],self.corner_2[1],self.corner_3[1],self.corner_4[1])
        self.footprint_y_max=max(self.corner_1[1],self.corner_2[1],self.corner_3[1],self.corner_4[1])
        self.footprint_box = [self.footprint_x_min,self.footprint_x_max,self.footprint_y_min,self.footprint_y_max]

    def compute_emission(self):
        emission_matrix = np.empty([self.image_height, self.image_width])
        for i in range(self.image_width):
            _, ray_line_matrix = self.camera.getRay([[i, j] for j in range(self.image_height)],normed=True)
            theta_matrix = np.arctan2(np.sqrt(ray_line_matrix[:,0]**2+ray_line_matrix[:,1]**2),-ray_line_matrix[:,2])*180.0/np.pi
            emission_matrix[:,i]=theta_matrix
        emission = self.camera.getTopViewOfImage(emission_matrix, self.footprint_box, scaling=self.gsd, do_plot=False)

        #if self.plot:
        #   self.plot('Emission',emission)


        self. write_worldfile_geotiff(emission,"emission","emission")

    def plot_image(self,title, image):
        cv2.imshow(title, image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def write_worldfile_geotiff(self,image:np.ndarray, description:str, suffix:str):
        print("Writinf file")
         # Word file ortho
        world_file_data = (self.gsd,0,0, -self.gsd, self.footprint_x_min, self.footprint_y_min)

        world_file_filename = f'{self.name}_{suffix}.tfw'
        with open(world_file_filename, 'w',encoding="utf-8")  as f_handle:
            np.savetxt(f_handle, world_file_data, fmt="%.3f")

        # Write orthophoto
        geotiff_filename = f'{self.name}_{suffix}.tif'
        tiff = TIFFimage(image, description=f"{self.file_name} {description}")
        tiff.write_file(geotiff_filename, compression=None) # or 'lzw'



    def compute_ortho(self):

        print("Compute orthophoto")

        im_array = io.imread(self.file_name)

        ortho = self.camera.getTopViewOfImage(im_array*self.thermal_gain, self.footprint_box, scaling=self.gsd, do_plot=False)

        if self.plot:
            self.plot_image("Orthophoto", ortho)

        self.write_worldfile_geotiff(ortho,"ortho", "orthophoto")


def main() -> None:
    """
    Create GEOtiff from DJI XT2 thermal Tiff
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
    parser.add_argument("-plot",help="Plot ortho",action='store_true')

    args = parser.parse_args()



    epsg_input = 'EPSG:4326' # WGS-84 [dd.mm.sssd]
    epsg_output = f'EPSG:{args.output_epsg}'


    # Transformation des coordonnées
    transformer = Transformer.from_crs(epsg_input,epsg_output)
    image_test=ImageClass(args.input_file,(args.sensor_width, args.sensor_height),gain=args.gain, plot=args.plot)

    image_test.local_longitude,image_test.local_latitude  = transformer.transform(image_test.latitude,image_test.longitude)
    print(f'{image_test.latitude=} {image_test.longitude=}\n{image_test.local_longitude=} {image_test.local_latitude=}')
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
