from kalmanfilter import KalmanFilter
from detector import detect
from detector import click
import cv2

class rocket:       # Klasė, kurioje bus kaupiamos raketos centro koordinatės, KF (Kalman Filter) naudojamas raketos sekimui bei nuspėjamų ir numatomų koordinačių nustatymui
    x_set = False
    def __init__(self, KF):     # metodas, kuris yra inicijuojamas sukurus raketos objektui. Reikalauja Kalman filtro įvesties, bei nustato pradinius kintamuosius
        self.KF = KF
        self.rocket_timer = 0
        self.xe = [0]
        self.ye = [0]
        self.x_p = [0]
        self.y_p = [0]
        self.xcen = 0
        self.ycen = 0
        self.updated = False

    def set_xcen(self,xcen):                    # metodas, skirtas įvesti centro X koordinates
        self.xcen = xcen

    def set_ycen(self,ycen):                    # metoda, skirtas įvesti centro Y koordinates
        self.ycen = ycen

    def get_xcen(self):                         # metodas, skirtas gražinti centro X koordinates
        return self.xcen

    def get_ycen(self):                         # metodas, skirtas gražinti centro Y koordinates
        return self.ycen

    def rocket_predict(self):                   # metodas, skirtas gražinti nuspėjamas X ir Y koordinates iš Kalman filtro
        self.x_p,self.y_p = self.KF.predict()
        return self.x_p, self.y_p

    def coords_update(self,xye):                # metodas, skirtas įvesti esamas raketos centro koordinates į Kalman filtrą ir
        self.xe,self.ye = self.KF.update(xye)   # gražinti numatomas X ir Y koordinates
        return self.xe,self.ye

    def getparam(self):
        return print(f"\n"
                     f"\n"
                     f" Object parameters:  "
                     f"\n"
                     f" xe {self.xe} ye {self.ye}"
                     f"\n"
                     f" xp {self.x_p} yp {self.y_p} "
                     f"\n"
                     f" xcen {self.xcen} ycen {self.ycen}"
                     f"\n")
    def timer(self):                # metodas skirtas padidinti neaktyvumo laikmatį viena reikšme
        self.rocket_timer+=1

    def get_time(self):             # metodas skirtas gražinti esamą neaktyvumo laikmačio reikšmę
        return self.rocket_timer

    def set_time(self):             # metodas skirtas atstatyti neaktyvumo laikmatį į pradinę vertę
        self.rocket_timer = 0

    def set_updated(self):          # metodas skirtas pakeisti indikatorių, reiškiantį, jog buvo atnaujintos koordinatės,bei
        self.updated = True         # bei atstatyti neaktyvumo laikmatį į pradinę vertę
        self.rocket_timer = 0

    def reset_updated(self):        # metodas skirtas atstatyti indikatorių į pradinę vertę
        self.updated = False

def main():

    HighSpeed = 100                 # vaizdo įrašo maksimali atkūrimo sparta
    ControlSpeedVar = 50           # kontrolinė vaizdo įrašo atkūrimo sparta, kai žemiausia: 1 - aukščiausia: 100
    debugMode = 1                   # kontrolinis kintamasis
    rocket_list = []                # raketos klasės objektų masyvas
    VideoCap = cv2.VideoCapture('8objects_test_video.mp4')       # vaizdo įrašo iš failo nuskaitymo algoritmas
    CameraCap = cv2.VideoCapture(0)                     # vaizdo įrašo iš kameros nuskaitymo algoritmas
    zone = 80                       # paieškos zonos dydis
    out = cv2.VideoWriter('outpy.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (1920, 1080)) # video write object
    for i in range(10):             # ciklas kuriantis 10 raketos objektų bei patalpinantis juos į masyvą
        KF = KalmanFilter(0.05, 4, 4, 4, 4, 4)
        rocket_list.append(rocket(KF))
        print(f" i  {i} ")
    i = 0

    while(True):
        ret, frame = VideoCap.read()                        # vaizdo įrašo nuskaitymas
        frame = cv2.resize(frame,(1920,1080))
        cv2.setMouseCallback("Rocket-Tracking", click)      # pelytės nuspaudimo funkcijaq
        centers, indexlist = detect(frame,debugMode)        # gražinami raketos kontūrų centrai iš detect funkcijos

        if(len(centers)>0):                                 # ciklas, tikrinantis buvo gražinti raketos kontūrų centrai

            for center in centers:                          # centro koordinačių iteracija
                x_coordinate = center[0]                    # išskyrimas x koordinatės
                y_coordinate = center[1]                    # išskrimoas y koordinatės

                for raketa in rocket_list:                  # Objektų iteracija
                    r_c_x = raketa.get_xcen()               # Gražinimos išsaugotos X koordinatės
                    r_c_y = raketa.get_ycen()               # Gražinimos išsaugotos Y koordinatės

                    if (int(x_coordinate) < int(r_c_x) + zone and int(x_coordinate) > int(r_c_x) - zone and \
                            int(y_coordinate) < int(r_c_y) + zone and int(y_coordinate) > int(r_c_y) - zone):
                        #ciklas tikrinantis ar gautos koordinatės patenka į objekte išsaugotų koordinačių paieškos zoną

                        raketa.set_xcen(x_coordinate)       # paleidžiamas metodas, įrašantis X koordinates
                        raketa.set_ycen(y_coordinate)       # paleidžiamas metodas, įrašantis Y koordinates
                        raketa.rocket_predict()             # paleidžiamas metodas, paleidžiantis Kalman filtro predict metodą
                        raketa.coords_update((x_coordinate, y_coordinate))      # paleidžiamas metodas, siunčiantis koordiantes į Kalman filtro update metodą
                        raketa.set_updated()                # paleidžiamas metodas, indikuojantis, kad buvo atnaujintos koordiantės
                        break

                    else:
                        if (r_c_x == 0 and r_c_y == 0):       # ciklas tikrinantis ar objektas yra "laisvas"
                            raketa.set_xcen(x_coordinate)     # jeigu objektas yra laisvas, išsaugomos x koordinatės
                            raketa.set_ycen(y_coordinate)     # ir y koordinatės

                            while i < 140:      # ciklas skirtas paspartinti taikinio judėjimą iš 0, 0 i x_cord & y_cord
                                raketa.coords_update((x_coordinate,y_coordinate))
                                raketa.rocket_predict()
                                i+=1
                            i = 0
                            break

                        else:
                            continue

        else:
            for Eraketa in rocket_list:             # objektų iteracija
                r_c_x = Eraketa.get_xcen()          # gražinimos išsaugotos X koordinatės
                r_c_y = Eraketa.get_ycen()          # gražinimos išsaugotos Y koordinatės

                if (r_c_x != 0 and r_c_y != 0):     # tikrinama ar objekte yra saugomos koordinatės
                    x_center = Eraketa.get_xcen()   # gražinimos išsaugotos X koordinatės
                    y_center = Eraketa.get_ycen()   # gražinimos išsaugotos Y koordinatės
                    (x_predicted, y_predicted) = Eraketa.rocket_predict()       # paleidžiamas metodas, gražinantis koordinates iš Kalman filtro predict metodo
                    (x_estimated, y_estimated) = Eraketa.coords_update((x_center, y_center))    # paleidžiamas metodas, siunčiantis esamas koordinates į update metodą
                                                                                                # ir gražinantis numatomas koordinates iš Kalman filtro update metodo

                    cv2.circle(frame, (int(x_center), int(y_center)), 15, (0, 191, 255), 2)     # ekrane apskritimu pažymimos esamos raketos koordinatės
                    cv2.circle(frame, (int(x_estimated), int(y_estimated)), 20, (0, 0, 255), 2) # ekrane apskritimu pažymimos numatomos raketos koordinatės
                    cv2.circle(frame, (int(x_predicted), int(y_predicted)), zone, (255, 0, 0), 2) # ekrane apskritimu pažymimos nuspėjamos raketos koordinatės
                    cv2.putText(frame, f"  Predicted Position ", (int(x_predicted + 15), int(y_predicted + 0)), 0, 0.5,(255, 0, 0), 2)   # ekrane tekstu įvardinamos nuspėjamos koordinatės

                else:
                    continue

        for RAKETA in rocket_list:          # objektų iteracija
            r_c_x = RAKETA.get_xcen()       # gražinimos išsaugotos X koordinatės
            r_c_y = RAKETA.get_ycen()       # gražinimos išsaugotos Y koordinatės

            if (r_c_x != 0) and (r_c_y != 0):      # tikrinama ar objekte yra saugomos koordinatės
                (x_predicted, y_predicted) = RAKETA.rocket_predict()      # paleidžiamas metodas, gražinantis koordinates iš Kalman filtro predict metodo
                x_center = RAKETA.get_xcen()    # gražinimos išsaugotos X koordinatės
                y_center = RAKETA.get_ycen()    # gražinimos išsaugotos Y koordinatės
                cv2.circle(frame, (int(x_center), int(y_center)), 15, (0, 191, 255), 2)     # ekrane apskritimu pažymimos esamos raketos koordinatės
                cv2.circle(frame, (int(x_predicted), int(y_predicted)), 25, (255, 0, 0), 2) # ekrane apskritimu pažymimos nuspėjamos raketos koordinatės
                (x_estimated, y_estimated) = RAKETA.coords_update((x_center,y_center))      # paleidžiamas metodas, siunčiantis esamas koordinates į update metodą
                                                                                            # ir gražinantis numatomas koordinates iš Kalman filtro update metodo
                cv2.circle(frame, (int(x_estimated),int(y_estimated)), 20, (0, 0, 255), 2)  # ekrane apskritimu pažymimos numatomos raketos koordinatės
                cv2.putText(frame, f"  Estimated Position ", (int(x_estimated + 15), int(y_estimated + 10)), 0, 0.5, (0, 0, 255), 2)   # ekrane tekstu įvardinamos numatomos koordinatės
                cv2.putText(frame, f"  Predicted Position ", (int(x_predicted + 15), int(y_predicted + 0)), 0, 0.5, (255, 0, 0), 2)    # ekrane tekstu įvardinamos nuspėjamos koordinatės
                cv2.putText(frame, f"  Measured Position  ", (int (x_center), int (y_center)), 0, 0.5, (0, 191, 255), 2)                # ekrane tekstu įvardinamos esamos koordinatės

            else:
                continue

        for r_timer in rocket_list:         # objektų iteracija
            time = r_timer.get_time()       # gražinamas objekto neaktyvumo laikmatis
            r_timer.timer()                 # neaktyvumo laikmatis padidinamas  vienetu

            if time > 40:                   # tikriname ar neaktyvumo laikmatis neviršijo ribos
                r_timer.set_xcen(0)         # atstatome objekte išsaugotas X koordinates į pradinę būseną
                r_timer.set_ycen(0)         # atstatome objekte išsaugotas Y koordinates į pradinę būseną
                r_timer.reset_updated()     # atstatome indikatorių į pradinę vertę
                r_timer.set_time()          # atstatome neaktyvumo laikmatį į pradinę vertę

            else:
                continue

        cv2.imshow('Rocket-Tracking', frame)        # funckija vaizdinei medžiagai parodyti ekrane

        if cv2.waitKey(2) & 0xFF == ord('q'):       # nustatome, kad paspaudus q klavišą būtų išjungta programa
            VideoCap.release()
            cv2.destroyAllWindows()
            break

        cv2.waitKey(HighSpeed-ControlSpeedVar+1)    # vaizdo atkūrimo sparta

        out.write(frame)                            # įrašomas video

if __name__ == "__main__":                          # paleidžiama programa
    main()