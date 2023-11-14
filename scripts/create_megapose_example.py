import os
import sys
from pathlib import Path
import shutil

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

def main():   
    args = sys.argv
    if len(args) < 2:
        print("missing argument")
        return
    print(args[1])

    datadir = os.environ.get('MEGAPOSE_DATA_DIR')
    if datadir == None:
        print("you need to set your MEGAPOSE_DATA_DIR environment variable")
        return

    #subdirectory ex: tiago/001
    if args[1].find("/"):
        print("OH")

    #whole directory ex: tiago
    if os.path.exists("./"+args[1]):
        print("ok "+args[1])
        get_subdir(datadir+"/examples/"+args[1].split("/")[-1], "./"+args[1])

#camera ?
def get_subdir(expath, frompath):
    #create nexessary dirs
    Path(expath).mkdir(parents=True,exist_ok=True)
    Path(expath + "/inputs").mkdir(exist_ok=True)
    Path(expath + "/meshes").mkdir(exist_ok=True)

    #get list of labels
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

def get_dir():
    return

if __name__ == '__main__':
    main()