import cv2
import numpy as np
import imutils

def click(event, x, y, flags, param):       # funkcija skirta atpažinti mygtuko paspaudimą ir gauti paspaudimo koordinates
    global targeting
    global targeting_x
    global targeting_y
    targeting_x = 0
    targeting_y = 0

    if event == cv2.EVENT_LBUTTONDOWN:      # kairiojo pelytes mygtuko paskaudimas
        targeting = True
        targeting_x = x
        targeting_y = y
        print ( "button down" )

    elif event == cv2.EVENT_LBUTTONUP:      # kairiojo pelytes mygtuko atleidimas
        targeting = False
        print ( "button up" )

    if targeting is True:
        print("targeting is active")

targeting = False
cv2.setMouseCallback("Rocket-Tracking", click)

def detect (frame,debugMode):           # vaizdo apdorojimo ir kontūrų aptikimo funkcija

    scale = 0.5
    indexnr = 0
    rgbed = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)       #
    gray = cv2.cvtColor(rgbed,cv2.COLOR_BGR2GRAY)       # vaizdo medžiagos pqavertimo pilkų atspalvių medžiaga
    blurred = cv2.GaussianBlur(gray,(3, 3), 0)          # vaizdo suliejimo funkcija
    img_edges=cv2.Canny(blurred, 20, 100, 1)            # kontūrų paryškinimo funkcija
    ret,img_thresh = cv2.threshold(img_edges, 245, 255, cv2.THRESH_BINARY)      # ribinis vaizdo pavertimas juodos ir baltos spalvos vaizdu
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (1, 1))         # branduolys būsimiems skaičiavimams
    opening = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)      # triukšmo ir smulkių kontūrų panaikinimas
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)        # triukšmo ir smulkių kontūrų panaikinimas
    contours= cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)       # uždarų kontūrų paieška
    contours = imutils.grab_contours(contours)

    min_area_thresh = 2500            # minimalaus ploto kriterijus
    max_area_thresh = 3000           # maksimalaus ploto kriterijus
    min_peri_thresh = 150            # minimalaus perimetro kriterijus
    max_peri_thresh = 300           # maksimalaus perimetro kriterijus
    min_radius_thresh = 20          # minimalaus apibrėžiančio apskritimo spindulio kriterijus
    max_radius_thresh = 60         # maksimalaus apibrėžiančio apskritimo spindulio kriterijus

    centers= []                     # aptiktų centrų masyvas
    indexlist = []

    for c in contours:              # kontūrų iteracija
        area = cv2.contourArea(c)   # kontūro ploto aptikimas
        area = float (area)
        peri = cv2.arcLength(c, True)       # kontūro perimetro apskaičiavimas
        epsilon = 0.01 * cv2.arcLength(c, True)         # maksimalus nuotolis nuo kontūro iki apytikslio kontūro
        approx = cv2.approxPolyDP(c, epsilon, True)     # apytikslis kontūras
        (x, y, w, h) = cv2.boundingRect(approx)         # apibrėžiantis stačiakampis
        aspectRatio = w / float(h)                      # kraštinių santykis
        (x_circle, y_circle), radius = cv2.minEnclosingCircle(approx)       # apibrėžiančio kontūro apskaičiavimas

        if (area > min_area_thresh) and (area < max_area_thresh) and (peri > min_peri_thresh) \
                and (peri < max_peri_thresh) and (radius>min_radius_thresh) and (radius< max_radius_thresh):   # ciklas tikrinantis ar kontūras atitinka kriterijus
            indexlist.append(indexnr)
            indexnr += 1
            centers.append(np.array([[x_circle],[y_circle]]))   # atitinkančius kontūrus pridedam į kontūrų centrų masyvą

    opening = cv2.resize(opening, (0, 0), None, scale, scale)   # keičiame apdorotos medžiagos rodomo lango dydį
    cv2.imshow('opening',opening)                               # rodom apdorotą medžiagą
    centers = sorted(centers, key = lambda x: x[1])             # rūšiuojam kontūrus nuo mažiausių Y koordinačių iki didžiausių
    return centers, indexlist                                   # gražiname centrų koordinačių masyvą /