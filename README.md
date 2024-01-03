# masterarbeit
## Overview
*  `/abgabe`: printable pdfs
*  `/txt`: latex project files
*  `/src`: rest
   *  `/code`: blender plugin & code for wall detailing
   *  `/env`: conda environment files to create the working environment for the projects
   *  `/figures for txt`: source files for the figures in the latex project
   *  `/models`:
      *  `/scenarios`: scenarios (1, 2, 3, 4) and the blender rendering project for figures in the latex project
      *  `/stuff to test stuff`: some test ifc files (some are fromhttps://www.ifcwiki.org/index.php?title=KIT_IFC_Examples)
   *  `/ontologies`: the base ontology and a temporary one that's being used to write and read to by the wall detailing project.

## Setup
### Conda Environment
```
 conda create -n masterarbeit python=3.10
 conda activate masterarbeit
 conda install -c conda-forge pythonocc-core=7.7.2
 conda install -c ifcopenshell -c conda-forge ifcopenshell
 conda install -c conda-forge quaternion
 conda install -c anaconda numpy
 conda install -c conda-forge owlready2
```
oder
```
conda env create -f /src/env/env.yml  # Windows
conda env create -f /src/env/env_universal.yml  # Linux
```
### Blender and Add-ons
* Download & Install Blender: https://www.blender.org/
* Download & Install BlenderBIM: https://blenderbim.org/
* How to install Blender Add-ons: https://docs.blender.org/manual/en/latest/editors/preferences/addons.html
* Now you can open and edit IFC Files in blender!
* Optional: Activate blender Add-on from this project: `/src/code/blender_add_on`

### Protégé
* Download & Install Protégé: https://protege.stanford.edu/
* Now you can inspect the ontologies in `/src/ontologies`!

## How to
* set the scenario and flags you want to use
* run wall_detailer.py
* Output Mesh: `/src/code/wall_detailing/output/output.stl`
* Output JSON: `/src/code/wall_detailing/output/output.json`
* Output Ontology: `/src/ontologies/temporary_working_env.rdf`
* Other outputs: `output/openings.stl` (all openings), `output/base.stl` (all walls that have been loaded), `output/output_NUMBER.stl` (partly built output)
