
#!!!! met iref ou -1? // ligne 631 change axe ? //758 ajoute une ligne??

# "switches"
bSTARTUP_TEST = False                            # indique si la sequence de test (vision et
                                                 # deplacement) doit etre effectuee
iVERSION = 3
# "constantes"
dbGO_GAIN   = 0.01                               # gain pour la commande "Go"
dbENTRAXE = 0.7
dbTOO_BIG_DISTANCE = 1000.0                      # valeur correspondant  une grande distance
dbDEFAULT_DISTANCE = 5.0                         # distance par defaut si on ne trouve pas la cible
dbRAYON_KUKA = 0.45                              # rayon du cercle que le "target" decrit autour du Kuka pour orienter l'axe 1
dbHAUTEUR_KUKA = 0.65                            # hauteur a laquelle positionner le "target" pour orienter l'axe 1
dbXK = 0.0                                       # abscisse KUKA par rapport a la base
dbYK = -0.25                                     # ordonnee KUKA par rapport a la base
# paremetre camera 
dbDEMI_ANGLE_CAM = 30                             # demi angle d'ouverture de la camera en °
MOBILE_BASE = "Robourrin"                         # nom de la base mobile de Robourrin dans la scene
LEFT_MOTOR = "moteur_gauche"                      # nom du moteur gauche dans la scene
RIGHT_MOTOR = "moteur_droit"                      # nom du moteur droit dans la scene
CAM_MOTOR = "moteur_tourelle"                     # nom du moteur de la tourelle dans la scene
CAMERA = "camera"                                 # nom du "vision sensor" associe a la tourelle dans la scene
COLLISION = "Robourrin"                           # nom de l'objet "collision" dans la scene
CYLINDRE = "cylindre"
CYLINDRE0 = "cylindre0"
CAPTEUR = "distance_sensor"                       # capteur de distance
TARGET_DUMMY = "target"                           # target dummy
SENSOR = "tool_detector"
TOOL = "tool"
NB_ACQUI  = 20
VREP_PORT   = 19997                               # port serveur de la remonte API COPPELIA
CAM_STARTUP_TIME = 0.5                            # duree d initialisation du systeme de vision
UPDATE_TIME = 0.1                                 # periode de mise a jour pour la prise de vue
RAYON_ROUE = 0.1                                  # rayon des roues du robot
# globales
gsiID = -1                                        # ID de connexion au serveur COPPELIA
gimg = 0                                          # image acquise
giCam = -1                                        # ID de la camera tourelle
giLeft = -1                                       # ID du moteur gauche
giRight = -1                                      # ID du moteur droit
giCamMotor = -1                                   # ID du moteur de la camerea
giCylindre = -1                                   # ID sur le handle du cylindre VERT pour estimer la distance du un premier temps
giCylindre0 = -1                                  # ID sur le handle du cylindre ROUGE pour estimer la distance du un premier temps
giCollision = -1                                  # ID de l'objet "collision" avec Robourrin
giSensor = -1                                     # ID du capteur de proximite du kuka
giRobourrin = -1                                  # ID du capteur de proximite de Robourrin     
giTarget = -1                                     # ID du target dummy
giBase = -1                                       # ID de la base mobile
giObject = -1                                    # handle sur l'objet capture
giTool = -1                                      # handle sur l'outil du robot
gModelIsDynamic = 0
iCount = 0
bOnGoing = True                                   # indique que la thread est en cours d'execution 
# tentative d'importation de la librairie V-REP : 
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
import time                                     # gestion du temps
import threading                                # pour la creation de taches cycliques
import math                                     # maths
import sys                                      # interface avec le shell
import cv2                                      # open CV
import numpy as np                              # calcul matriciel et cie.
import matplotlib.pyplot as plt                 # trace de courbes facon MATLAB
#&&&&&&&&&&&&&&&&&&&&&
# aide de ce programme
#&&&&&&&&&&&&&&&&&&&&&
def usage(szPgmName):
  print(szPgmName + '< Adresse IP du serveur V-REP>')
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# premier appel pour intialiser le systeme de vision : 
# aproche "force brutale"...
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def InitVisionSystem(siID, iCam):
  siError, imgResolution, imgData = sim.simxGetVisionSensorImage(siID, iCam, 0, sim.simx_opmode_streaming )
  if(siError != sim.simx_return_ok) and (siError != sim.simx_return_novalue_flag):
    print('InitVisionSystem() : ERREUR -->appel a simxGetVisionSensorImage()')
    print('                     code d erreur V-REP = ' + str(siError))
    return(-1)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# "wrapper" pour convertir l'image V-REP en quelque chose de 
#  manipulable par OpenCV
# IN : 
#   imgRes  : resolution de l'image [lignes, colonnes]
#   imgData : donnees image (en ligne)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def VREP2CV2ImageWrapper( imgRes, imgData ):
    imgShape = [imgRes[0],imgRes[1],3]
    cvImg = np.zeros(imgShape, dtype = np.uint8 )
    index = 0
    for i in range(imgRes[0]):
        for j in range(imgRes[1]):
            # V-REP travaille en RGB, et OpenCV en BGR...
            # il faut de plus inverser l'ordre des lignes
            cvImg[imgRes[0] -1 - i,j,2] = imgData[index]
            cvImg[imgRes[0] -1 - i,j,1] = imgData[index+1]
            cvImg[imgRes[0] -1 - i,j,0] = imgData[index+2]
            index = index + 3
    return cvImg
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction d'acquisition d'image depuis V-REP
# (faire un appel en debut de script principal a InitVisionSystem()
# IN :
#   iCam : "handle" V-REP de la camera depuis laquelle faire l'acquisition
# OUT : 
#   image couleur au format OpenCV
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def GrabImageFromCam( siID, iCam ):
  siError, imgResolution, imgData = sim.simxGetVisionSensorImage(siID, iCam, 0, sim.simx_opmode_buffer )
  if( siError != sim.simx_return_ok ) and (siError != sim.simx_return_novalue_flag):
      print('GrabImageFromCam() : ERREUR --> appel a simxGetVisionSensorImage()')
      print('                     code d errer V-REP = ' + str(siError))
      return []
  # Appel du "Wrapper" pour convertir au format
  # RGB OpenCV : 
  cvImg = VREP2CV2ImageWrapper(imgResolution, imgData)
  # fini
  return cvImg
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# declenchement d'une mesure de distance avec le capteur
# a ultrasons
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def GetDistanceMeasurement(detecte):
    if detecte == "sensor_kuka": # on détecte avec Kuka
        iError,bDetected,Point,iObject,surface=sim.simxReadProximitySensor(gsiID,giSensor,sim.simx_opmode_blocking)
        if bDetected:
            dPoint = np.array(Point)
            dbDist = np.linalg.norm(dPoint)
            print('distance = ' + str(dbDist))
            return True, dbDist, iObject
        return False, -1.0, -1
    if detecte =="sensor_robourrin" :  # on détecte avec Robourrin
        iError,bDetected,Point,iObject,surface=sim.simxReadProximitySensor(gsiID,giRobourrin,sim.simx_opmode_blocking)
        if bDetected:
            dPoint = np.array(Point)
            dbDist = np.linalg.norm(dPoint)
            print('distance = ' + str(dbDist))
            return True, dbDist
        return False, -1.0
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# obtention de la position (generalisee) de la base 
# mobile du robot 
# IN : 
#   iRef    : handle sur l'objet associe au referentiel
#             utilise pour reperer l'objet  
# OUT :
#   [x,y,z] : position (globale) du repere de la base
#             mobile
#   [a,b,c] : orientation (globale) du repere de base
#             mobile (X est dans la direction d'avance
#             du robot).                                
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def GetMobileBasePosition(iRef):  
  #............................
  # recuperation de la position
  #............................
  siError, Pos =  sim.simxGetObjectPosition(siID, giTarget, iRef ,sim.simx_opmode_blocking)
  if (siError != sim.simx_return_ok ):
    print('GetMobileBasePosition() : ERREUR ---> appel a simxGetObjectPosition().\n')
    print('code d erreur V-REP = ', str(siError) )
    return [],[]
  #.............................
  # recuperation de l'orientation
  #..............................
  siError, Ori =  sim.simxGetObjectOrientation(siID, giTarget, iRef ,sim.simx_opmode_blocking)
  if (siError != sim.simx_return_ok ):
    print('GetMobilePosition() : ERREUR ---> appel a simxGetObjectOrientation().\n')
    print('code d erreur V-REP = ', str(siError) )
    return Pos,[]
  # OK
  return Pos, Ori
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# obtention de la position (generalisee) de la base 
# mobile du robot 
# IN : 
#   iRef    : handle sur l'objet associe au referentiel
#             utilise pour reperer l'objet  
# OUT :
#   [x,y,z] : position (globale) du repere de la base
#             mobile
#   [a,b,c] : orientation (globale) du repere de base
#             mobile (X est dans la direction d'avance
#             du robot).                                
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def GetObjectRelativePositionAndOrientation(iObject, iRef):  
  #............................
  # recuperation de la position
  #............................
  siError, Pos =  sim.simxGetObjectPosition(siID, iObject, iRef ,sim.simx_opmode_blocking)
  if (siError != sim.simx_return_ok ):
    print('GetMobileBasePosition() : ERREUR ---> appel a simxGetObjectPosition().\n')
    print('code d erreur V-REP = ', str(siError) )
    return [],[]
  #.............................
  # recuperation de l'orientation
  #..............................
  siError, Ori =  sim.simxGetObjectOrientation(siID, iObject, iRef ,sim.simx_opmode_blocking)
  if (siError != sim.simx_return_ok ):
    print('GetMobilePosition() : ERREUR ---> appel a simxGetObjectOrientation().\n')
    print('code d erreur V-REP = ', str(siError) )
    return Pos,[]
  # OK
  return Pos, Ori
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# activation de la rotation de la tourelle
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def RotateCameraContinuous( siID, hCamMotor, dbVelocity):
    #....................................*/
    # Application des vitesses au moteur */
    #....................................*/
    siError = sim.simxSetJointTargetVelocity(siID, hCamMotor, dbVelocity, sim.simx_opmode_blocking)
    if ((siError != sim.simx_return_ok) and (siError != sim.simx_return_novalue_flag)):
        print('Go() : ERREUR ---> appel a simxSetJointTargetVelocity() depuis RotateCameraContinuous()')
        print('code d erreur V-REP = ' + str(siError))
        return(-1)
    return 0
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# positionnement de la camera a une orientation donnee
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def SetCamOrientation( siID, hCamMotor, dbOrientation):
    #....................................*/
    # Application des vitesses au moteur */
    #....................................*/
    siError = sim.simxSetJointTargetPosition(siID, hCamMotor, (dbOrientation / 180.0)*math.pi, sim.simx_opmode_blocking)
    if ((siError != sim.simx_return_ok) and (siError != sim.simx_return_novalue_flag)):
        print('Go() : ERREUR ---> appel a simxSetJointTargetVelocity() depuis RotateCameraContinuous()')    
        print('code d erreur V-REP = ' + str(siError))
        return(-1)
    # TEST
    OrienteKuka(( 1.0 + dbOrientation / 180.0)*math.pi)
    return 0
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# application des vitesses angulaires aux moteurs
# gauche et droit
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def SetBaseMotorsVelocities( siID, iLeft, dbVelLeft, iRight, dbVelRight):
    siError = sim.simxSetJointTargetVelocity(siID, iLeft, dbVelLeft, sim.simx_opmode_blocking)
    if ((siError != sim.simx_return_ok) and (siError != sim.simx_return_novalue_flag)):
        print('Go() : ERREUR ---> appel a simxSetJointTargetVelocity() depuis SetBaseMotorsVelocities() 1')
        print('code d erreur V-REP = ' + str(siError))
        return(-1)
    siError = sim.simxSetJointTargetVelocity(siID, iRight, dbVelRight, sim.simx_opmode_blocking)
    if ((siError != sim.simx_return_ok) and (siError != sim.simx_return_novalue_flag)):
        print('Go() : ERREUR ---> appel a simxSetJointTargetVelocity() depuis SetBaseMotorsVelocities() 2') 
        print('code d erreur V-REP = ' + str(siError))
        return(-1)
    return 0
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# exemple de fonction permettant de realiser l'acquisition d'une
# image de maniere cyclique
# Remarque : 
# cette fonction est executee par une thread, ce qui explique
# l'usage du mot clef 'global' devant le nom des variables globales
# utilisee ici
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def CyclicGrabbing():
  global iCount           
  global gimg
  global gsiID
  global giCam
  global bOnGoing
  global NB_ACQUI

  gimg = GrabImageFromCam(gsiID, giCam)
  print('aquisition num ' + str( iCount))
  iCount = iCount + 1

  if iCount < NB_ACQUI:
    threading.Timer(UPDATE_TIME, CyclicGrabbing).start()
  else:
    bOnGoing = False
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# obtention de la position (relative) de la base 
# mobile du robot   
# OUT :
#   [x,y,z] : position (globale) du repere de la base
#             mobile
#   [a,b,c] : orientation (globale) du repere de base
#             mobile (X est dans la direction d'avance
#             du robot).                                
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def GetMobileBaseRelativePosition(iRef):  
  #............................
  # recuperation de la position
  #............................
  siError, Pos =  sim.simxGetObjectPosition(siID, iBaseHandle, iRef ,sim.simx_opmode_blocking)
  if (siError != sim.simx_return_ok ):
    print('GetMobileBasePosition() : ERREUR ---> appel a simxGetObjectPosition().\n')
    print('code d erreur V-REP = ', str(siError) )
    return [],[]
  #.............................
  # recuperation de l'orientation
  #..............................
  siError, Ori =  sim.simxGetObjectOrientation(siID, iBaseHandle, iRef ,sim.simx_opmode_blocking)
  if (siError != sim.simx_return_ok ):
    print('GetMobilePosition() : ERREUR ---> appel a simxGetObjectOrientation().\n')
    print('code d erreur V-REP = ', str(siError) )
    return Pos,[]
  # OK
  return Pos, Ori
#!!!!!
# TODO (1):
# ecrire une fonction permettant de faire avancer le robot 
# vers l'avant ou l'arriere d'une distance donnee
# (le robot s'arrete une fois la distance parcourue)
# Go( dbDist, dbVmax, dbEpsilon, dbGain, dbTimeStep)
# IN :
# dbDist = la "distance" a parcourir (en m)
#          si > 0 : vers l'avant
#          si < 0 : vers l'arrier
# dbVmax = la vitesse maximale autorisee
# dbEpsilon = l'erreur en distance toleree. Le robot
#             peut s'arreter de la difference avec la
#             distance theorique a parcourir vaut dbEpsilon
# dbGain = valeur du gain (si on utilise une loi de commande de
#          type proportionnelle)
# dbTimeStep = le pas temporel utilise pour la mise a jour de
#              la commande
#----------------------------------------------------------
# REMARQUE IMPORTANTE : si le gain est trop important, il 
# y a un risque de depasser la distance a parcourir, dans 
# ce cas, le robot poursuivra sa route en accelerant (en 
# utilisant dbEpsilon).
# alternative : s'arreter des que la distance parcourue
# devient superieure a la distance a parcourir. 
# le mieux : gerer les deux conditions (si on ne declenche
# que sur le depassement, il y a alors un risque que si on
# ne depasse pas, le fonction ne "retourne" jamais... ! 
#----------------------------------------------------------
# Ce qu'on doit constater : 
# - les distances parcourues sont plus longues que les 
#   distances reelles...
# - origine du probleme : on suppose respectees les periodes
#   d'echantillonnage gerees par time.sleep(). Elles sont
#   de fait largement depassees ! Solution --> affiner la
#   prise en compte du temps en evaluant le temps qui s'est
#   "reellement" ecoule (du point de vue du simulateur)
#   entre deux mises a jour ---> fonction Go2
#!!!
# TODO 2.1 : 
# re-ecrire la fonction Go en utilisant la connaissance de 
# la position absolue du robot
# RQ : ETRE MOINS STRICT SUR EPSILON ET AUGMENTER (UN PEU !) LE GAIN
# IN
# dbDist : distance a parcourir
# dbVmax : vitesse maximale autorisee
# dbEpsilon : tolerance sur la distance a parcourir
# dbGain : gain pour la commande proportionnelle
# dbMinDist : distance a l'obstacle minimale declenchant l'arret
# dbTimeStep : ~ periode de mise a jour de la commande
# OUT :
# 0, d : distance parcourue dans rencontrer d'obstable (distance = d)
# 1, d : obstacle rencontre (distance parcourue ~ d)
#!!!
def Go5( dbDist, dbVmax, dbEpsilon, dbGain, dbMinDist, dbTimeStep):
  dbDparcourue = 0.0
  # recuperation de la position initiale  
  while True:
      Pos, Ori = GetMobileBasePosition(-1)   #doit laisser?????????????????
      if len(Pos) > 0:
          break
  # on itere jusqu'a ce qu'on ait parcouru approximativement la distance
  while abs( dbDparcourue - abs(dbDist)) > dbEpsilon:
      # MESURE de la distance au cylindre 
      bResult, dbDistTarget = GetDistanceMeasurement("sensor_robourrin")
      if bResult == False:
          dbDistTarget = dbDEFAULT_DISTANCE
          print('cible non detectee, distance par defaut = ' + str(dbDEFAULT_DISTANCE))
      print('distance MESUREE a la cible = ' + str(dbDistTarget))
      if dbDistTarget < dbMinDist:
          # on arrete le deplacement si on est assez pres de la cible : 
          SetBaseMotorsVelocities( gsiID, giLeft, 0.0, giRight, 0.0)
          return 1, dbDparcourue, dbDistTarget
      dbV = dbGain * abs( dbDparcourue - abs(dbDist))
      # limitation de la vitesse d'avance
      if dbV > dbVmax:
          dbV = dbVmax
      # pour info : 
      dbError = abs( dbDparcourue - abs(dbDist))
      print('erreur = ' + str(dbError) + ' vitesse = ' + str(dbV) + ' m/s distance = ' + str(dbDparcourue))
      # mise a jour de la distance parcourue 
      while True:
        PosCur, OriCur = GetMobileBasePosition(-1)
        if len(PosCur) > 0:
            break
      M0 = np.array( Pos )        # position initiale
      M1 = np.array( PosCur )     # position actuelle
      dbDparcourue = np.linalg.norm( M0 - M1)
      if dbDist < 0:
        dbV = -dbV
      # application des vitesses :
      Wroue = dbV /  RAYON_ROUE  
      SetBaseMotorsVelocities( gsiID, giLeft, Wroue, giRight, Wroue)
      time.sleep( dbTimeStep/2)
      # mise a jour de l'indice de l'iteration
    # en quittant, on fixe les vitesses a 0 : 
  SetBaseMotorsVelocities( gsiID, giLeft, 0.0, giRight, 0.0)
  return 0, dbDparcourue, dbDistTarget # pas de collision
  
#!!!!
# TODO (2): 
# ecrire une fonction permettant de faire tourner le robot
# dans le sens trigo ou inverse d'un angle donnee
# (le robot s'arrete une fois la rotation achevee)
# Turn( dbAngle, dbWmax, dbEpsilon, dbGain, dbTimeStep)
# IN :
# dbAngle = l'angle a parcourir (en radians)
#          si > 0 : sens trigonometrique
#          si < 0 : sens horaire
# dbWmax = la vitesse angulaire maximale autorisee
# dbEpsilon = l'erreur en angle toleree. Le robot
#             peut s'arreter de la difference avec l'
#             angle theorique a decrire vaut dbEpsilon
# dbGain = valeur du gain (si on utilise une loi de commande de
#          type proportionnelle)
# dbTimeStep = le pas temporel utilise pour la mise a jour de
#              la commande
#!!!!
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
#!!!!
# TODO 2.2 : 
# re-ecrire la fonction Turn en utilisant la connaissance de 
# l'orientation absolue du robot.
# L'ASTUCE : a la position et a l'orientation initiale du robot
# on "pose" un dummy qui servira de reference angulaire lors de 
# la rotation
# Il faut utiliser la position relative a ce repere
# RQ : ETRE MOINS STRICT SUR EPSILON ET AUGMENTER (UN PEU !) LE GAIN
#!!!!
def Turn2( dbAngle, dbWmax, dbEpsilon, dbGain, dbTimeStep):
  dbWL = 0.0
  dbWR = 0.0
  dbAparcourue = 0.0
  # recuperation de la position initiale  
  while True:
      Pos, Ori = GetMobileBasePosition(-1)
      if len(Ori) > 0:
          break
  # on cree le dummy :
  iError,iDummy = sim.simxCreateDummy(gsiID,0.02,None, sim.simx_opmode_blocking)
  # on place et oriente le dummy
  iError = sim.simxSetObjectPosition(gsiID,iDummy,-1,Pos,sim.simx_opmode_blocking)
  iError = sim.simxSetObjectOrientation(gsiID,iDummy,-1,Ori,sim.simx_opmode_blocking)
  # on itere jusqu'a ce qu'on ait parcouru approximativement l'angle
  while abs( dbAparcourue - abs(dbAngle)) > dbEpsilon:
      # mise a jour de l'angle parcourue
      # (on tourne autour de Z) 
      while True:
        PosCur, OriCur = GetMobileBaseRelativePosition(iDummy)
        if len(Ori) > 0:
            break
      dbW = dbGain * abs( dbAparcourue - abs(dbAngle))
      # limitation de la vitesse de rotation
      if dbW > dbWmax:
          dbW = dbWmax
      # d ou la vitesse lineaire des roues 
      dbVLin = dbW * 0.5 *dbENTRAXE
      # d ou la vitesse angulaire des roues
      dbWroue = dbVLin / RAYON_ROUE
      M0 = np.array( Ori )
      M1 = np.array( OriCur )
      dbAparcourue = abs( OriCur[2])
      if dbAngle < 0:
        dbWR = dbWroue
        dbWL = -dbWroue
      else:
        dbWR = -dbWroue
        dbWL = dbWroue
      # pour info : 
      dbError = abs( dbAparcourue - abs(dbAngle))
      print('erreur = ' + str(dbError) + ' vitesse = ' + str(dbWroue) + 'rad/s angle = ' + str(dbAparcourue))
      # application des vitesses :
      SetBaseMotorsVelocities( gsiID, giLeft, dbWL, giRight, dbWR)
      time.sleep( dbTimeStep)
  # en quittant, on detruit le dummy on fixe les vitesses a 0 :
  iError = sim.simxRemoveObject(gsiID,iDummy,sim.simx_opmode_blocking) 
  SetBaseMotorsVelocities( gsiID, giLeft, 0.0, giRight, 0.0)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# determination de la distance focale de la camera 
# (en pixels horizontaux)
# Rq : le demi angle d'ouverture horizontale de la
#      camera est donne en degres.
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def GetCamFocal( dbAngle, dbWidth):
    dbF = dbWidth / (2.0 * math.tan((dbAngle/180.0)*math.pi))
    return dbF
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# determination de l'azimut d'un point image
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def Azimut( dbX, dbY, dbF, dbW):
    dbTan = (0.5 * dbW - dbX ) / dbF
    dbAzimut = (math.atan(dbTan) / math.pi)*180.0
    return dbAzimut
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# determination de l'azimut du cylindre vu de la camera, 
# s'il y a lieu (s'il est visible)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def AzimutCameraCylindre( dbAngle, bGoR=True):
    # declenchement d'un acquisition
    img1 = GrabImageFromCam(gsiID, giCam)
    cv2.imshow("ORIGINAL", img1)
    cv2.waitKey(10)
    # determination du centre de gravite du plan vert
    #iNbPixels, Center = CoG(img1, 180, 1)
    iNbPixels, Center = CoGGreenOrRed(img1, 160, 60, bGoR)
    xc = Center[0]
    yc = Center[1]
    # si a trouve qqchose, on affiche le centre :
    if xc > 0:
        print("nombre de pixels sur la cible = " + str(iNbPixels))

        img2 = cv2.copyTo(img1,np.ones(img1.shape, np.uint8))
        # on trace une croix centree sur le CoG
        pt1 = (int(xc) , 0)
        pt2 = (int(xc), img2.shape[1] - 1)
        cv2.line(img2, pt1, pt2,(0,255,255))
        cv2.imshow("TARGET_DUMMY", img2)
        cv2.waitKey(10)
        # calcul de l'angle d'azimut local
        dbF = GetCamFocal( 30.0, img1.shape[0])
        dbAz = Azimut( xc, yc, dbF, img1.shape[0])
    else:
        dbAz = -180.0   # pas coherent, pour test de validite
    return xc, dbAz

#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# calcul du centre de gravite associe a un plan image
# IN : 
# imgIn : image d'entree (format OpenCV) 
# dbSeuil : seuil pour la "binarisation"
# iPlan : 'plan' image (0,1, ou 2)
# bGoR : si True -> cylindre vert, si False cylindre rouge
# OUT :
# [xCoG, yCoG] : coordonnees image du centre de gravite
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def CoGGreenOrRed( imgIn, dbSeuil, dbSeuilMax, bGoR=True):
    # recuperation des dimensions de l'image
    iImgSize = imgIn.shape
    iNl = iImgSize[0]       # nombre de lignes ( hauteur de l'image)
    iNc = iImgSize[1]       # nombre de colonnes (largeur de l'image)
    iNp = iImgSize[2]       # nombre de plan couleur
    # intialisation
    dbXcog = 0.0
    dbYcog = 0.0
    iNbPixels = 0
    # calcul
    for i in range(iNl):
        for j in range(iNc):
            iB = imgIn[i,j,0]
            iG = imgIn[i,j,1]
            iR = imgIn[i,j,2]

            if bGoR == True:
                if iG > dbSeuil and iB < dbSeuilMax and iR < dbSeuilMax:
                    dbXcog += j
                    dbYcog += i
                    iNbPixels += 1
            else:
                if iR > dbSeuil and iB < dbSeuilMax and iG < dbSeuilMax:
                    dbXcog += j
                    dbYcog += i
                    iNbPixels += 1
    # coordonnees finales
    if iNbPixels > 0:
        dbXcog = dbXcog / iNbPixels
        dbYcog = dbYcog / iNbPixels
    else:
        dbXcog = -1
        dbYcog = -1
    # fini
    print('xc = ' + str( dbXcog))
    return iNbPixels, [dbXcog, dbYcog]
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# recherche du cylindre en faisant tourner la tourelle
# on retourne l'azimut du cylindre s'il a ete trouve 
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def RechercheCylindre( dbAngleStep, bGoR=True):
    dbAngleCouvert = 0.0            # angle total couvert lors de la recherche
    while dbAngleCouvert < 360.0:
        SetCamOrientation(gsiID, giCamMotor, dbAngleCouvert )
        iFound, dbAz = AzimutCameraCylindre(30.0, bGoR) # (1) recherche du cylindre dans l'image  (iFound > 0 si trouve)
                                                        # (2) calcul de l'azimut "local" (vu de la camera) si cylindre trouve
        if iFound > 0:
            print("\a\a\a\a")                           # bip victorieux....
            # determination de l'azimut global par rapport a Robourrin.
            rPos, rOri = GetMobileBaseRelativePosition( giCamMotor) # donne position et orientation relative base robourrin / tourelle
            dbRelAngle = -rOri[2]                           # seule la rotation autour de z nous interesse ici
            dbGlobAz = (dbRelAngle * 180)/math.pi + dbAz    # on ajoute la rotation tourelle / base robourrin a l'azimut local..
                                                            # ..vu de la camera
            print('azimut gobal = ' + str( dbGlobAz))       # Affichage pour verification (degres)
            return iFound, dbGlobAz                         # on retourne iFound > 0 (cylindre trouve) et l'azimut GLOBAL
        else:
            # rotation de la camera de la valeur de l'angle 
            # d'ouverture
            dbAngleCouvert += dbAngleStep                   # si on n'a pas trouve, on continu le scan camera 
            SetCamOrientation(gsiID, giCamMotor, dbAngleCouvert )
    # pas trouve...
    return -1, -180.0

#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction de recherche et de rencontre du cylindre
# Version plus "adaptative" pour les parcours
# rectilignes.
# IN : 
#   dbAngleStep : pas de recherche de la cible avec
#                 la camera
#   dbGoStep : pas de deplacement entre 2 scans
#   dbMinDist : distance minimale toleree en l'absence
#               de choc..
# OUT : 
#   -1 : cible non detectee
#    0 : atteinte de la cible
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def BourineCylindre2( dbAngleStep, dbGoStep, dbMinDist,bGoR):
    dbDist = dbTOO_BIG_DISTANCE
    while dbDist > dbMinDist:
        # scan de la cible : 
        iResult, dbAz = RechercheCylindre(dbAngleStep, bGoR)
        # on s'oriente vers la cible si on l'a trouvee :
        if iResult > 0:
            Turn2((dbAz/180.0)*math.pi, 0.2, 0.05, 0.1, 0.1)
            # on recale la camera dans l'axe d'avance
            SetCamOrientation(gsiID, giCamMotor, 0.0)  #!!!!!!!!!!!!!! change l'axe 
            # estimation de la distance a la cible : 
            GetDistanceMeasurement("sensor_robourrin")
            # calcul de la distance au cylindre 
            rPos, rOri = GetMobileBaseRelativePosition( giCylindre)
            # petit "piege" a eviter : ne pas prendre en compte z...
            rPos[2] = 0.0     #!!!!!!!!!!!!!! elle a commenté
            v1 = np.array(rPos)
            dbDistTarget = np.linalg.norm(v1)
            iError, dbDistTarget, iDetected = GetDistanceMeasurement()
            if iError == False:
                dbDistTarget = dbDEFAULT_DISTANCE
            # on avance d'un metre vers la cible (les moteurs sont orientes a l'envers...)
            #iError, dbDistParcourue, dbDist = Go4(-0.5*dbDistTarget, 0.5, 0.02, 0.15, dbMinDist, 0.1)
            iError, dbDistParcourue, dbDist = Go5(-0.5*dbDistTarget, 0.25, 0.05, 0.3, dbMinDist, 0.1) # >>> diminue vitesse et aumente epsilon et gain pour gagner en précision
            if iError == 1:
                return 0
        else:
            return -1 # la cible n'a pas ete trouvee ou a disparu
    return 0    # cible atteinte a la precision voulue...
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# deplacement de l'outil du robot
# IN
# iRef : referentiel par rapport auquel on deplace (-1 : global)
# tPos : position cible = [xT,yT,zT]
# tOri : orientation cible = [aT,bT,cT]
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def PlaceToolAt( iRef, tPos):
    iError = sim.simxSetObjectPosition(gsiID,giTarget,iRef,tPos,sim.simx_opmode_blocking)
    if iError != sim.simx_return_ok:
        print('ERREUR : MoveToolTo() ---> appel a simxSetObjectPosition() ')
        print('         valeur de retour = ' + str(iError))
        return -1
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# Positionnement du dummy mobile avec le robot pour que la base
# du Kuka pivote d'un angle theta autour de l'axe 1
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def OrienteKuka( dbAngle ):
    dbAngle = dbAngle % (2.0 * math.pi)
    dbX = dbXK + dbRAYON_KUKA * math.cos( dbAngle + 0.5 * math.pi)
    dbY = dbYK + dbRAYON_KUKA * math.sin( dbAngle + 0.5 * math.pi)
    dbZ = dbHAUTEUR_KUKA
    # on place le "target" relativement a la base de Robourrin
    PlaceToolAt( giBase, [dbX,dbY,dbZ])
    return 0
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# deplacement de l'outil du robot
# IN
# iRef : referentiel par rapport auquel on deplace (-1 : global)
# tPos : position cible = [xT,yT,zT]
# tOri : orientation cible = [aT,bT,cT]
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def MoveToolTo( iRef, tPos,tOri):
    iError = sim.simxSetObjectPosition(gsiID,giTarget,iRef,tPos,sim.simx_opmode_blocking)
    if iError != sim.simx_return_ok:
        print('ERREUR : MoveToolTo() ---> appel a simxSetObjectPosition() ')
        print('         valeur de retour = ' + str(iError))
        return -1
    iError = sim.simxSetObjectOrientation(gsiID,giTarget,iRef,tOri,sim.simx_opmode_blocking)
    if iError != sim.simx_return_ok:
        print('ERREUR : MoveToolTo() ---> appel a simxSetObjectOrientation() ')
        print('         valeur de retour = ' + str(iError))
        return -1
    return 0
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# deplacement de l'outil a une position absolue donnee
# IN : 
#  tPos       : position cible
#  dbVel      : vitesse lineaire souhaitee
#  dbTimeStep : pas temporel pour le deplacement
# OUT : 
#  aPos       : position reellement atteinte
#  aOri       : orientaion rellement atteinte
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def PTP( tPos, dbVel, dbTimeStep, iRef):      # !!!!voir si pas direct remplcé irep par -1????
    # obtention de la position et l'orientation courante
    # de l'outil
    while True:
        cPos, cOri = GetMobileBaseRelativePosition(iRef)
        if (len(cPos) > 0) and (len(cOri) > 0):
            break
    # calcul des variations 
    DPos = np.array(tPos) - np.array(cPos)
    # calcul du nombre de pas a realiser a priori
    npDist = np.linalg.norm(DPos)
    dbT = npDist / dbVel
    # nombre d'iterations
    iNbIter = int(dbT / dbTimeStep)
    dPos = DPos / iNbIter
    # initialisation
    Pos = cPos
    # deplacement
    for i in range(iNbIter):
        Pos += dPos
        MoveToolTo( iRef, Pos.tolist(), cOri)
        time.sleep( dbTimeStep * 0.25)
    return Pos.tolist(), cOri
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction de capture d'un objet "en contact"
# avec le capteur d'effort
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def Capture():
    global giObject
    global gModelIsDynamic
    # on ne capture que si un objet n'est pas deja capture : 
    if giObject == -1:
        # on commence par lire l'etat du capteur d'effort
        iState, dbDist, giObject = GetDistanceMeasurement("sensor_kuka")
        if iState == True:  # on a detecte quelque chose
            # capture de l'etat dynamique / statique de l'objet : 
            iError, gModelIsDynamic =sim.simxGetModelProperty(gsiID,giObject,sim.simx_opmode_blocking)
            # on rend l'objet statique : 
            iError =sim.simxSetModelProperty(gsiID,giObject, sim.sim_modelproperty_not_dynamic, sim.simx_opmode_blocking)
            # il faut capturer l'objet : (cette fois-ci, on le rend "enfant" de l'outil du robot)
            print('CAPTURE')
            #iError = sim.simxSetObjectParent(gsiID,giObject,giTarget,True,sim.simx_opmode_blocking)
            iError = sim.simxSetObjectParent(gsiID,giObject,giTool,True,sim.simx_opmode_blocking)
        else:
            print('PAS DE DETECTION...')
    else:
        print('PREHENSEUR OCCUPE...')
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction de liberation d'un objet 
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def Release():
    global giObject
    global gModelIsDynamic
    # on ne capture que si un objet n'est pas deja capture :    #!!!!!ajoute une ligne ??
    if giObject != -1:
        p,o = GetObjectRelativePositionAndOrientation(giObject, -1)
        # il faut liberer l'objet :
        iError = sim.simxSetObjectParent(gsiID,giObject,-1,False,sim.simx_opmode_blocking)
        # on restaure l'etat de l'objet : 
        iError =sim.simxSetModelProperty(gsiID,giObject, gModelIsDynamic , sim.simx_opmode_blocking)
        #iError =sim.simxSetModelProperty(gsiID,giObject, 0 , sim.simx_opmode_blocking)
        # on doit lui reassigner sa position dans l'espace de travail
        iError = sim.simxSetObjectPosition(gsiID,giObject,-1,p,sim.simx_opmode_blocking)
        if iError != sim.simx_return_ok:
            print('ERREUR : Release() ---> appel a simxSetObjectPosition() ')
            print('         valeur de retour = ' + str(iError))
            return -1
        iError = sim.simxSetObjectOrientation(gsiID,giObject,-1,o,sim.simx_opmode_blocking)
        if iError != sim.simx_return_ok:
            print('ERREUR : Release() ---> appel a simxSetObjectOrientation() ')
            print('         valeur de retour = ' + str(iError))
            return -1
            
##########################
# point d'entree du script 
##########################
argc = len(sys.argv)
if( argc == 1 ):
  usage(sys.argv[0])
  exit()
#.........................................
# tentative de connexion au serveur V-REP 
#.........................................
siID = sim.simxStart(sys.argv[1], VREP_PORT, 1, 1, 10000, 50)
if( siID < 0):
  print('ERREUR : main() ---> appel a simxStart() : impossible de se connecter a ' +  sys.argv[1])
  print('         valeur de retour = ' + str(siID))
  exit()  
print("connexion au serveur COPPELIA etablie.")
gsiID = siID
#recuperation des "handles" sur le robot, les moteurs et la camera
#..........................................
# recuperation du handle sur la base mobile 
#..........................................
siErrorCode, iBaseHandle = sim.simxGetObjectHandle(siID, MOBILE_BASE, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  sim.simxFinish(siID)
  exit()
print("lien a la base mobile OK (ID = " + str(iBaseHandle) + ")")
giBase = iBaseHandle
#............................................
# recuperation du handle sur le moteur gauche 
#............................................
siErrorCode, iLeftMotor = sim.simxGetObjectHandle(siID, LEFT_MOTOR, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  sim.simxFinish(siID)
  exit()
print("lien au moteur gauche OK (ID = " + str(iLeftMotor) + ")")
giLeft = iLeftMotor
#............................................
# recuperation du handle sur le moteur droit 
#............................................
siErrorCode, iRightMotor = sim.simxGetObjectHandle(siID, RIGHT_MOTOR, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  sim.simxFinish(siID)
  exit()
print("lien au moteur droit OK (ID = " + str(iRightMotor) + ")")
giRight = iRightMotor
#....................................................
# recuperation du handle sur le moteur de la tourelle 
#....................................................
siErrorCode, iCamMotor = sim.simxGetObjectHandle(siID, CAM_MOTOR, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  sim.simxFinish(siID)
  exit()
print("lien au moteur de la tourelle OK (ID = " + str(iCamMotor) + ")")
giCamMotor = iCamMotor
#.......................................
# recuperation des handles sur la camera 
#.......................................
siErrorCode, iCam = sim.simxGetObjectHandle(siID, CAMERA, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  sim.simxFinish(siID)
  exit()
print("lien a la camera OK (ID = " + str(iCam) + ")")
giCam = iCam
#...............................................
# recuperation du handle sur l'objet "collision"  >>>> cylindre vert
#...............................................
siErrorCode, giCylindre = sim.simxGetObjectHandle(siID, CYLINDRE, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
    print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
    print('         code de retour V-REP = ' + str(siErrorCode))
    sim.simxFinish(siID)
    exit()
print("lien a la collision OK (ID = " + str(giCylindre) + ")")
#...............................................
# recuperation du handle sur l'objet "collision"  >>>> cylindre rouge
#...............................................
siErrorCode, giCylindre0 = sim.simxGetObjectHandle(siID, CYLINDRE0, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
    print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
    print('         code de retour V-REP = ' + str(siErrorCode))
    sim.simxFinish(siID)
    exit()
print("lien a la collision OK (ID = " + str(giCylindre0) + ")")
#...........................................
# recuperation du handle sur le target dummy 
#...........................................
siErrorCode, giTarget = sim.simxGetObjectHandle(siID, TARGET_DUMMY, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  sim.simxFinish(siID)
  exit()
print("lien au target dummy OK (ID = " + str(giTarget) + ")")
if iVERSION > 2:
    #......................................
    # recuperation du handle sur le capteur
    #......................................
    siErrorCode, giRobourrin = sim.simxGetObjectHandle(siID, CAPTEUR, sim.simx_opmode_blocking)
    if( siErrorCode != sim.simx_error_noerror ):
        print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
        print('         code de retour V-REP = ' + str(siErrorCode))
        sim.simxFinish(siID)
        exit()
    print("lien au capteur OK (ID = " + str(giRobourrin) + ")")
#.............................................
# recuperation du handle sur l'outil du robot' 
#.............................................
siErrorCode, giTool = sim.simxGetObjectHandle(siID, TOOL, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  sim.simxFinish(siID)
  exit()
print("lien a l outil du robot OK (ID = " + str(giTool) + ")")
#...............................................
# recuperation du handle sur le capteur d'effort 
#...............................................
siErrorCode, giSensor = sim.simxGetObjectHandle(siID, SENSOR, sim.simx_opmode_blocking)
if( siErrorCode != sim.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  sim.simxFinish(siID)
  exit()
print("lien au capteur d effort OK (ID = " + str(giSensor) + ")")
#..............................
# mise en rotation de la camera
# (par exemple pour prendre une vue
# panoramique)
#..............................
#___________________________________________________________________
if bSTARTUP_TEST:
    RotateCameraContinuous(siID, iCamMotor, (9.0 / 180.0) * math.pi)
else:
    # pour etre sur que le robot soit bien a l'arret au demarrage
    SetBaseMotorsVelocities( gsiID, iLeftMotor, 0.0, iRightMotor, 0.0)
#___________________________________________________________________
InitVisionSystem(siID, iCam)          # a n'appeler qu'une fois a priori
time.sleep(CAM_STARTUP_TIME)
#....................................
# tentative de grabing
# a l'aide dune thread s'executant de
# maniere cyclique 
#....................................
#________________________________________________________________________________
if bSTARTUP_TEST:
    CyclicGrabbing()                    # IMPORTANT : le code de CyclicGrabbing()
                                        # s'execute en // de la thread principale
    # on verifie que la thread est terminee (pas tres elegant...)
    # s'execute en // de la thread de grabbing
    while True:
        cv2.imshow("RENDER", gimg)
        cv2.waitKey(20)
        aPos, aOri = GetMobileBasePosition()
        if len(aPos) > 0: # il peut arriver que la liste retournee soit vide ! 
            strX =  '{:.3f}'.format(aPos[0])
            strY =  '{:.3f}'.format(aPos[1])
            strZ =  '{:.3f}'.format(aPos[2])
            print('position: ( ' + strX + ',' + strY + ',' + strZ + ')')
        if len(aOri) > 0: # il peut arriver que la liste retournee soit vide !
            strA =  '{:.3f}'.format(aOri[0])
            strB =  '{:.3f}'.format(aOri[1])
            strC =  '{:.3f}'.format(aOri[2])
            print('orientation: ( ' + strA + ',' + strB + ',' + strC + ')')
        if bOnGoing == False:
            break
    # arret de la rotation de la camera 
    RotateCameraContinuous(siID, iCamMotor, 0.0)
    # on fait decrire au robot un arc de cercle 
    SetBaseMotorsVelocities(siID, iLeftMotor, (30/180)*math.pi, iRightMotor, (-10/180)*math.pi )
    time.sleep(5)
#_______________________________________________________________________________________________
# on arrete le robot 
SetBaseMotorsVelocities(siID, iLeftMotor, 0, iRightMotor, 0 )

# recherche et rencontre du cylindre vert : 
BourineCylindre2(15.0, 1.0, 0.1, True)     # on s'approche a 1m
SetBaseMotorsVelocities(gsiID, iLeftMotor, 0, iRightMotor, 0 )
# rotation 1/2 de tour a gauche pour mettre le kuka face au robot: 
#Turn2( math.pi, 0.5, 0.01, 0.2, 0.1)   # on est moins strict sur epsilon (passe de 0.008 dans l'exemple à 0.01) et on augmente le gain (0.1 à 0.2)
# on se place à 30cm du cylindre en marche arrière (pour ne pas taper dedans avec BourrineCylindre)
#Go5(0.7,0.5,0.01,0.2,0.1,0.1)


# puis recherche et rencontre du cylindre rouge :
#BourineCylindre2(15.0, 1.0, 1.0, False)
#SetBaseMotorsVelocities(siID, iLeftMotor, 0, iRightMotor, 0 )

#AzimutCameraCylindre(30.0)
# on avance de 2 metres avec Go : 
#Go2( 2, 0.5, 0.01, 0.1, 0.1)
#Go2( -2, 0.5, 0.01, 0.1, 0.1)
# # dessin d'un carre d'1,5 m de cote : 
# # test : avancees de 5 m
# Go3( 5, 0.5, 0.02, 0.15, 0.1) 
# # # rotation 1/4 de tour a gauche : 
# Turn2( (90/180) * math.pi, 0.5, (0.5/180)*math.pi, 0.1, 0.1)
# # # test : avancees de 1.5 m
# Go3( 5, 0.5, 0.02, 0.15, 0.1)
# # # rotation 1/4 de tour a gauche : 
# Turn2( (90/180) * math.pi, 0.5, (0.5/180)*math.pi, 0.1, 0.1)
# # # test : avancees de 1.5 m
# Go3( 5, 0.5, 0.02, 0.15, 0.1)
# # # rotation 1/4 de tour a gauche : 
# Turn2( (90/180) * math.pi, 0.5, (0.5/180)*math.pi, 0.1, 0.1)
# # # test : avancees de 1.5 m
# Go3( 5, 0.5, 0.02, 0.15, 0.1)
# # # rotation 1/4 de tour a gauche : 
# Turn2( (90/180) * math.pi, 0.5, (0.5/180)*math.pi, 0.1, 0.1)
#..............................
# deconnexion du serveur V-REP 
#..............................
sim.simxFinish(gsiID)
print("deconnexion du serveur.")



