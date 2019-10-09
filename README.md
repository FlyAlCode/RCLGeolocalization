## Introduction
This is the implementation for our paper "Road network based fast geolocalization", where the 
road networks in the aerial images are used to localize the images in the reference geographical 
coordinate system by registering the road map to the reference road map.

## Build
The project depends on the OpenCV, shapelib and the nanoflann. 
You can install the nanoflann following introdcution in: https://github.com/jlblancoc/nanoflann.
And you should change the opencv path in the top cmakelist file to your own opencv path.
The shapelib is used to read .shp files, and convert them into images. We provide the lib in the directory ‘third_lib’.

After all the dependences have been install, you can build the project:

$ mkdir build

$ cd build

$ cmake ../

$ make ./

## Run
1. Detect all road intersections and compute their tangents with:

    $ ./make_reference_map [shp_file] [cross_file_name]
    
    shp_file --- the reference road vector map save in .shp format
    
    cross_file_name --- the file to save the road intersections and theri tangents

2. Perform geolocalization with:

    $ ./rcl_geolocalize [map_file_name] [query_img_dir] [offset_x] [offset_y] [result_file]

    map_file_name --- the file to save the road intersections, which can be get with step one above

    query_img_dir --- the directory where the query road map are 

    offset_x/offset_y --- the offset between the global geographical coordination and the reference area

    result_file --- the file to save the geolocalization result