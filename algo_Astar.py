# -*- coding: utf-8 -*-
"""
Created on Tue Apr 10 10:52:05 2018

@author: Thibault et Thomas
"""

import json
import numpy as np
import math

#lecture du fichier contenant les obstacles
json_data=open('Description_Environnement.js')
data_environnement = json.load(json_data)
#print (data_environnement['obstacles'])
json_data.close()

#lecture du fichier du chemin
json_data=open('Evolution_Base_Roulante.js')
data_base=json.load(json_data)
#print (data_base)
json_data.close()

#matrice représentant l'environement
matriceObstacle=np.zeros((45,45)) 
for obstacle in data_environnement['obstacles']:
    matriceObstacle[obstacle['coordonnees'][0]['centre']['x']/25,obstacle['coordonnees'][0]['centre']['y']/25]=1
    
print("matrice avec obstacles")
print(matriceObstacle)

coordRobot=np.array([data_base['depart']['coordonnees']['x']/25,data_base['depart']['coordonnees']['y']/25])
print ("depart du robot en :",coordRobot)

#recupération des coordonnées des étapes
matriceEtapes=[]
for i in range(0,len(data_base['etapes'])):
    matriceEtapes.append([data_base['etapes'][i]['coordonnees']['x'],data_base['etapes'][i]['coordonnees']['y']])
matriceEtapes.append([data_base['arrivee']['coordonnees']['x'],data_base['arrivee']['coordonnees']['y']])
#coordonnées position de tir du robot
matriceEtapes.append([data_base['position-tir_cible']['coordonnees']['x'],data_base['position-tir_cible']['coordonnees']['y']])

matriceEtapes=np.array(matriceEtapes)/25
print("etapes :")
print (matriceEtapes)

case_valide=[]
case_pasOptimale=[]
case_fausse=[]
#depart_x=0;depart_y=0
depart_x=data_base['depart']['coordonnees']['x']/25
depart_y=data_base['depart']['coordonnees']['y']/25

x_case=depart_x
y_case=depart_y

#iteration de chaque étape
for k in range(0,len(matriceEtapes)):    
    arrivee_x=matriceEtapes[k][0];arrivee_y=matriceEtapes[k][1] 
    #boucle tant qu'on est pas arrivé à l'étape      
    while(not((depart_x == arrivee_x) and (depart_y == arrivee_y))):  
        list_tri=[]
        #parcours des cases autour de la case actuelle
        for i in range(-1,2):
            for j in range(-1,2):
                if(depart_x+i<0 or depart_y+j<0):
                    case_fausse.append([depart_x+i,depart_y+j])
                    continue
                if(matriceObstacle[depart_x+i][depart_y+j]==1):
                    case_fausse.append([depart_x+i,depart_y+j])
                    continue
                if(not((depart_x+i)==depart_x and (depart_y+j)==depart_y)):
                    #calcul du cout du chemin
                    coutCalcul=math.sqrt((arrivee_x-(depart_x+i))**2+(arrivee_y-(depart_y+j))**2)+math.sqrt((i**2)+(j**2))
                    list_tri.append([depart_x+i,depart_y+j,coutCalcul])
                    case_pasOptimale.append([depart_x+i,depart_y+j,coutCalcul])
        
        
        coutfinal=list_tri[0][2]
        x_case=list_tri[0][0]
        y_case=list_tri[0][1]
        #selection du chemin le moins couteux
        for i in range(0,len(list_tri)):
            if (list_tri[i][2]<coutfinal):
                coutfinal=list_tri[i][2]
                x_case=list_tri[i][0]
                y_case=list_tri[i][1]
        
        case_valide.append([x_case,y_case,coutfinal])
        case_pasOptimale.remove([x_case,y_case,coutfinal])
        depart_x=x_case
        depart_y=y_case

print ""
print("case fausse",case_fausse)
print ("chemin empreinte [x,y,cout]:",case_valide)

matrice_chemin=matriceObstacle
for case in case_valide:
    matrice_chemin[case[0],case[1]]=2  
print(matrice_chemin)

trajectoire=[]
angle=0
trajectoire.append([data_base['depart']['coordonnees']['x']/25,data_base['depart']['coordonnees']['y']/25,data_base['depart']['angle']])
for i in range(1,len(case_valide)):
    if (float(case_valide[i][0]-case_valide[i-1][0])!=0):    
        angle=math.atan(float(case_valide[i][1]-case_valide[i-1][1])/float(case_valide[i][0]-case_valide[i-1][0]))
       
    if(float(case_valide[i][0]-case_valide[i-1][0])==0):
        angle=math.pi/2
    
    angle=angle*180/math.pi
    trajectoire.append([case_valide[i][0],case_valide[i][1],angle])
    if (trajectoire[-1][2]==trajectoire[-2][2]):
        trajectoire.remove(trajectoire[-2])
    
        

print (trajectoire)

#écriture de la trajectoire dans un fichier
fichier=open("trajectoire.txt","w")
for i in range(0,len(trajectoire)):    
    if(str(trajectoire[i][2])=="-0.0"):
        trajectoire[i][2]=180.0
    fichier.write("G X:"+str(trajectoire[i][0]*25)+" Y:"+str(trajectoire[i][1]*25)+" A:"+str(trajectoire[i][2])+"\n")
    if (trajectoire[i][0]==(data_base['arrivee']['coordonnees']['x']/25) and trajectoire[i][1]==(data_base['arrivee']['coordonnees']['y']/25)):        
        fichier.write("G X:"+str(trajectoire[i][0]*25)+" Y:"+str(trajectoire[i][1]*25)+" A:"+str(float(data_base['arrivee']['angle']))+"\n")
fichier.write("G X:"+str(trajectoire[-1][0]*25)+" Y:"+str(trajectoire[-1][1]*25)+" A:"+str(data_base['position-tir_cible']['angle'])+"\n")
fichier.close


