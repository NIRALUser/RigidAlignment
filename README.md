# Rigid Alignment 


## Description

Rigid alignment of the landmarks on the unit sphere: the input models share the same unit sphere and their landmarks are defined as spacial coordinates (x,y,z) of the input model.
Outputs are a new aligned sphere for each model.


## Build

#### Requirements 

Building RigidAlignment requires to have built previously:

* SlicerExecutionModel


###### Linux or MacOSX 

Consider this tree of repertories:
```
~/Project/RigidAlignment
         /RigidAlignment-build
```

Start a terminal.
First change your current working directory to the build directory ```RigidAlignment-build```
```
cd ~/Project/RigidAlignment-build
```

Generate the project using ```cmake```
```
cmake -DVTK_DIR:PATH=path/to/VTK -DITK_DIR:PATH=path/to/ITK -DSlicerExecutionModel:PATH=path/to/SlicerExecutionModel ../RigidAlignment
make
```


## Usage

If the inputs models (vtk files) and their landmarks files (fcsv files) are isolated in two distinct folders: 

```
./RigidWrapper --mesh [<std::string> input models directory] --landmark [<std::string> input fiducial files directory] 
--sphere [<std::string> common unit sphere] --output [<std::string> output directory] 

```

Or vtk files and fcsv files can be listed in a CSV file as following ```VTK file, FCSV file```. In that case, the command line is now: 

```
./RigidWrapper --inputCSV [<std::string> input CSV file] --sphere [<std::string> common unit sphere] 
--output [<std::string> output directory] 

```

NOTE: The reconstruction of new surface models from the results of RigidAlignment is possible via SurfRemesh (git@github.com:pdedumast/SurfRemesh.git).

## Licence

See LICENSE.txt for information on using and contributing.