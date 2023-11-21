import os
import sys
from pathlib import Path
import shutil
import argparse

import logging
logging.basicConfig()
logger = logging.getLogger('create_megapose_example')
logger.setLevel(logging.INFO)

#infos importantes:
#ca fonctionne par sous dossier : un dossier principal, sous dossiers dedans avec des exemples
#possibilite de recuperer tout le dosssier ou non
#donnees de camera a la racine de chaque dossier
#le nom d'un mesh doit corresondre au label de l'objet a reconnaitre (fichier object_data.json)


#si lance a la racine : cree les dossiers pour tous les exemples
#si lance sur un dossier d'exemple precis : cree uniquement l'exemple concerne

#chaque dossier contient : 
#   une image
#   les donnees de position de l'objet
#   les donnees de position de la camera du roobot
#   ~les donnees de la camera --> a part pour eviter les duplicats ? probablement
#   le(s) label(s) de(s) objet(s)
#   la bbox correspondante
#   les meshs correspondants --> dossier a part pour eviter les duplicats

#creer les dossiers input et meshes
#bouger l'image et l'appeler  image_rgb.png
#recuper le label de l'objet en question
#creer les sous dossiers meshes correspondants aux labels
#aller chercher les meshs correspondant aux labels et les mettre dedans
#creer le fichier de camera avec les donnees que l'on a
#creer le fichier d'input en recuperant les labels et les bbox
#camera ?

def get_dir(name):
    #create nexessary dirs
    frompath = os.path.dirname(os.path.realpath(__file__))+f'/../tiago/{name}'
    datadir = os.environ.get('MEGAPOSE_DATA_DIR')
    if datadir == None:
        print("you need to set your MEGAPOSE_DATA_DIR environment variable")
        return

    expath = datadir + '/examples'
    Path(expath).mkdir(parents=True,exist_ok=True)
    Path(expath + "/inputs").mkdir(exist_ok=True)
    Path(expath + "/meshes").mkdir(exist_ok=True)

    #copy obj_data_file
    shutil.copy(frompath+"/obj_data.json", expath+"/inputs/obj_data.json")

    #get list of labels #read from obj_data
    labels = ["tiago_01","tiago_00"]

    #copy image
    if not os.path.exists(frompath+"/image.png"):
        print("where image?")
        return
    shutil.copy(frompath+"/image.png", expath+"/image.png") #TODO: *.png

    #create necessary files
    shutil.copy(frompath+"/../camera_data.json", expath+"/camera_data.json")
    shutil.copy(frompath+"/object_data.json", expath+"/inputs/object_data.json")

    #import meshes
    for l in labels:
        dest = expath + "/meshes/"+l
        meshpath = "./meshes/"+l+".truc" #TODO: l.*
        Path(dest).mkdir(exist_ok=True)
        shutil.copy(meshpath, dest)

def main():
    parser = argparse.ArgumentParser('Get megapose results')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()

    if not args.name:
        logger.error('Please provide an example name : --name <example_name>')
        return
    
    get_dir(args.name)

if __name__ == '__main__':
    main()