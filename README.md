# Installation via pip

Create a virtual environment 

```
python3 -m venv .venv # Install with compatible maximum version of Python (Requires-Python >=3.7,<3.11)
source .venv/bin/activate
```

# install requirements

```
pip install  -r requirements.txt  
````


# Usage

## Compute orthophoto for file DJI_0560.tiff using EPSG:4471  
````
./DjiXt2Lib.py  DJI_0560.tiff 4471
````

## Compute orthophoto and emission  for file DJI_0560.tiff using EPSG:4471  
````
./DjiXt2Lib.py  DJI_0560.tiff 4471 -emission
````
## Compute  emission  for file DJI_0560.tiff using EPSG:4471  
````
./DjiXt2Lib.py  DJI_0560.tiff 4471 -emission -no_ortho


## Compute orthophoto AND emission  for file DJI_0560.tiff using EPSG:4471  
````
./DjiXt2Lib.py  DJI_0560.tiff 4471 -emission
````

## Options
-plot : Display orthophoto and/or emission images
-emission :Compute emission image
-no_ortho : Do not compute orthophoto
