import cv2
import numpy as np
import lgpio as GPIO
import time
import serial
import math
import serial.tools.list_ports
import sys
import smbus2


import threading
from threading import Thread

h = GPIO.gpiochip_open(0)



# =========================
# CONFIG (NÃO ALTERADO)
# =========================
Kp = 2 #2
Kd = 0.8 #0.8
tempo_sem_linha = 0 
VEL_BASE = 30
VEL_MAX  = 45 #50
VEL_MIN  = -VEL_MAX #-50
VEL_GAP = 35
LIMIAR   = 80
DEADZONE = 6 #4
AREA_MIN = 6000
girando = False

IN1, IN2, ENA = 17, 18, 12
IN3, IN4, ENB = 22, 23, 13
BUTTON = 24

contador_direita = 0
contador_esquerda =  0
contador_180 = 0
# =========================
# ESTADO
# =========================
tempo_gap = 0
ultimo_erro = 0
ultimo_tempo = time.time()


green_min = np.array([40, 50, 45])
green_max = np.array([85, 255, 255])

black_min     = np.array([0, 0, 0])
black_max_top = np.array([90, 90, 90])
black_max_bot = np.array([135, 135, 135])

MIN_GREEN_AREA = 2500
kernal = np.ones((3, 3), np.uint8)
# =========================
# SERIAL (NÃO ALTERADO)
# =========================
PORTA_FIXA = "/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0"



def detectar_porta():
    # 1) Tenta a porta com ID fixo (mais confiável quando disponível)
    try:
        s = serial.Serial(PORTA_FIXA, 115200, timeout=0.0)
        print(f"[Serial] Conectado via ID fixo: {PORTA_FIXA}")
        return s
    except serial.SerialException:
        print("[Serial] ID fixo não encontrado, buscando...")

    # 2) Varre todas as portas e filtra por fabricante/descrição FTDI
    PALAVRAS_FTDI = ["ftdi", "ft232", "usb serial converter", "ftb6spl3"]
    portas = serial.tools.list_ports.comports()

    for p in portas:
        desc = f"{p.description} {p.manufacturer or ''} {p.hwid or ''}".lower()
        if any(palavra in desc for palavra in PALAVRAS_FTDI):
            try:
                s = serial.Serial(p.device, 115200, timeout=0.1)
                print(f"[Serial] FTDI encontrado em: {p.device} — {p.description}")
                return s
            except serial.SerialException:
                continue

    # 3) Última tentativa: qualquer ttyUSB ou ttyACM disponível
    for p in portas:
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            try:
                s = serial.Serial(p.device, 115200, timeout=0.0)
                print(f"[Serial] Usando porta genérica: {p.device} — {p.description}")
                return s
            except serial.SerialException:
                continue

    # Falhou em tudo — mostra o que existe pra facilitar o debug
    print("\n[ERRO] Nenhuma porta serial encontrada!")
    print("Portas disponíveis no sistema:")
    for p in portas:
        print(f"  {p.device:20s} | {p.description}")
    if not portas:
        print("  (nenhuma porta detectada — verifique o cabo USB)")
    sys.exit(1)


ser = detectar_porta()

time.sleep(2.0)
ser.reset_input_buffer()
ser.write(b"START\n")
time.sleep(2.0)           # ← espera o Arduino processar o START
ser.reset_input_buffer()  # ← limpa qualquer lixo que voltou
print("Handshake enviado pro Arduino")



# =========================
# GPIO INIT
# =========================
GPIO.gpio_claim_input(h, BUTTON, GPIO.SET_PULL_UP)
pins = [IN1, IN2, IN3, IN4, ENA, ENB]

for p in pins:
    GPIO.gpio_claim_output(h, p)

GPIO.tx_pwm(h, ENA, 50, 0)
GPIO.tx_pwm(h, ENB,50, 0)
# =========================
# FUNÇOES DE COMUNICACAO COM NANO (NÃO ALTERADO)
# =========================

# =========================
# MPU6050 DIRETO NA RASP
# =========================
MPU_ADDR    = 0x68
GYRO_XOUT_H = 0x43
bus = smbus2.SMBus(1)

ang_gyro          = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_offset       = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_ultimo_tempo = time.time()

def _mpu_write(reg, val):
    bus.write_byte_data(MPU_ADDR, reg, val)

def _mpu_read_word(reg):
    hi  = bus.read_byte_data(MPU_ADDR, reg)
    lo  = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (hi << 8) | lo
    if val >= 0x8000: val -= 65536
    return val

def _normalizar(ang):
    while ang >  180: ang -= 360
    while ang < -180: ang += 360
    return ang

def _calibrar(amostras=200):
    gx = gy = gz = 0
    for _ in range(amostras):
        gx += _mpu_read_word(GYRO_XOUT_H)
        gy += _mpu_read_word(GYRO_XOUT_H + 2)
        gz += _mpu_read_word(GYRO_XOUT_H + 4)
        time.sleep(0.005)
    gyro_offset["X"] = gx / amostras
    gyro_offset["Y"] = gy / amostras
    gyro_offset["Z"] = gz / amostras



import threading
_gyro_lock = threading.Lock()

def resetar_giroscopio():
    global ang_gyro, gyro_ultimo_tempo
    with _gyro_lock:
        ang_gyro = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        gyro_ultimo_tempo = time.time()


def angulo_x(): return int(ang_gyro["X"])
def angulo_y(): return int(ang_gyro["Y"])
def angulo_z(): return int(ang_gyro["Z"])

def _gyro_thread():
    global ang_gyro, gyro_ultimo_tempo
    while True:
        try:
            now = time.time()
            with _gyro_lock:
                dt = now - gyro_ultimo_tempo
                gyro_ultimo_tempo = now

            if dt > 0.5:
                time.sleep(0.005)
                continue

            gx = (_mpu_read_word(GYRO_XOUT_H)     - gyro_offset["X"]) / 131.0
            gy = (_mpu_read_word(GYRO_XOUT_H + 2) - gyro_offset["Y"]) / 131.0
            gz = (_mpu_read_word(GYRO_XOUT_H + 4) - gyro_offset["Z"]) / 131.0

            with _gyro_lock:
                ang_gyro["X"] = _normalizar(ang_gyro["X"] + gx * dt)
                ang_gyro["Y"] = _normalizar(ang_gyro["Y"] + gy * dt)
                ang_gyro["Z"] = _normalizar(ang_gyro["Z"] + gz * dt)
        except:
            pass
        time.sleep(0.005)

_mpu_write(0x6B, 0x00)
time.sleep(0.1)
_mpu_write(0x1B, 0x00)
time.sleep(0.1)
print("[MPU6050] Calibrando...")
_calibrar()
resetar_giroscopio()
print("[MPU6050] Pronto")
Thread(target=_gyro_thread, daemon=True).start()

# =========================
# SERIAL — só sonar (F e E)
# =========================
dados = {"F": 9999.0, "E": 9999.0}

def serial_thread():
    while True:
        try:
            linha = ser.readline().decode().strip()
            if not linha: continue
            for parte in linha.split(","):
                chave, valor = parte.split(":")
                if chave in dados:
                    dados[chave] = float(valor)
        except:
            pass

Thread(target=serial_thread, daemon=True).start()

def distancia_frente():
    return dados["F"]

def distancia_esquerda():
    return dados["E"]



# =========================
# MOTORES (NÃO ALTERADO)
# =========================
def aplicar_motor(in1, in2, pwm_pin, vel):
    vel = max(min(vel, VEL_MAX), VEL_MIN)

    if vel > 0:
        GPIO.gpio_write(h, in1, 0)
        GPIO.gpio_write(h, in2, 1)

    elif vel < 0:
        GPIO.gpio_write(h, in1, 1)
        GPIO.gpio_write(h, in2, 0)

    else:
        GPIO.gpio_write(h, in1, 0)
        GPIO.gpio_write(h, in2, 0)

    duty = min(abs(vel), 100)
    GPIO.tx_pwm(h, pwm_pin, 50, duty)

def mover(velL, velR):
    aplicar_motor(IN2, IN1, ENA, velL)
    aplicar_motor(IN4, IN3, ENB, velR)

def parar():
    mover(0, 0)

# =========================
# CAMERA

camera_x = 160
camera_y = 120

def detectar_camera():
    for i in range(10):
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"[Camera] Conectada em /dev/video{i}")
                return cap
            cap.release()
    return None

cap = detectar_camera()
while cap is None:
    print("[Camera] Tentando novamente em 2s...")
    time.sleep(2)
    cap = detectar_camera()

cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(3, camera_x)
cap.set(4, camera_y)




# =========================
# ESTADO
# =========================

rodando = False
ultimo_estado = 1
# =========================
# VISÃO (TUDO JUNTO)
# =========================
def visao(frame):
    h, w = frame.shape[:2]
    roi = frame[int(h*0.1):h, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, LIMIAR, 255, cv2.THRESH_BINARY_INV)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx = None

    if contours:
        main = max(contours, key=cv2.contourArea)
        M = cv2.moments(main)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])

    return cx, contours, roi, mask

# =========================
# GAP
# =========================
def gap(contours):
    global tempo_gap

    if len(contours) == 0:
        tempo_gap += 1
        return True

    maior = max(contours, key=cv2.contourArea)

    if cv2.contourArea(maior) < AREA_MIN:
        tempo_gap += 1
        return True
    
    tempo_gap = 0
    return False

def angulo_linha(frame):
    cx, contours, roi, mask = visao(frame)
    
    if not contours:
        return None
    
    maior = max(contours, key=cv2.contourArea)
    
    if cv2.contourArea(maior) < AREA_MIN:
        return None
    
    [vx, vy, x0, y0] = cv2.fitLine(maior, cv2.DIST_L2, 0, 0.01, 0.01)
    
    angulo = math.degrees(math.atan2(vx, vy))
    
    # Desenha a linha ajustada na ROI
    h, w = roi.shape[:2]
    t = max(h, w)
    x1 = int(x0 - vx * t)
    y1 = int(y0 - vy * t)
    x2 = int(x0 + vx * t)
    y2 = int(y0 + vy * t)
    cv2.line(roi, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(roi, f"{angulo:.1f} graus", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    return angulo

def alinhar_linha(frame):
    global tempo_sem_linha

    angulo = angulo_linha(frame)

    if angulo is None:
        tempo_sem_linha += 1

        if tempo_sem_linha < 20:
            mover(-30, -30)
        else:
            parar()

        return False

    tempo_sem_linha = 0

    erro_angulo = 90 - angulo

    if abs(erro_angulo) < 2:
        parar()
        return True

    ajuste_linha = erro_angulo * 0.5
    ajuste_linha = np.sign(ajuste_linha) * (abs(ajuste_linha) + 25)
    ajuste_linha = np.clip(ajuste_linha, -50, 50)

    mover(ajuste_linha, -ajuste_linha)

    return False

def check_black(black_around_sign, i, green_box, black_image):
    gb_y = green_box[green_box[:, 1].argsort()]
    marker_height = int(gb_y[-1][1] - gb_y[0][1])
    sign_y = int(gb_y[2][1])
    black_around_sign[i, 4] = sign_y

    # Baixo
    y1, y2 = sign_y, min(sign_y + int(marker_height * 0.8), camera_y)
    x1, x2 = int(min(gb_y[2][0], gb_y[3][0])), int(max(gb_y[2][0], gb_y[3][0]))
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 0] = 1

    # Cima
    y1, y2 = max(int(gb_y[1][1]) - int(marker_height * 0.8), 0), int(gb_y[1][1])
    x1, x2 = max(int(min(gb_y[0][0], gb_y[1][0])), 0), max(int(max(gb_y[0][0], gb_y[1][0])), 0)
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 1] = 1

    gb_x = green_box[green_box[:, 0].argsort()]

    # Esquerda
    y1, y2 = int(min(gb_x[0][1], gb_x[1][1])), int(max(gb_x[0][1], gb_x[1][1]))
    x1, x2 = max(int(gb_x[1][0]) - int(marker_height * 0.8), 0), int(gb_x[1][0])
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 2] = 1

    # Direita
    y1, y2 = int(min(gb_x[2][1], gb_x[3][1])), int(max(gb_x[2][1], gb_x[3][1]))
    x1, x2 = int(gb_x[2][0]), min(int(gb_x[2][0]) + int(marker_height * 0.8), camera_x)
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 3] = 1

    return black_around_sign


def determine_turn_direction(black_around_sign, green_centers):
    votes_left  = 0
    votes_right = 0

    mid_x = (max(green_centers) + min(green_centers)) / 2 if len(green_centers) >= 2 else camera_x / 2

    for i, row in enumerate(black_around_sign):
        if row[1] == 1 and row[2] == 1 and row[0] == 0 and row[3] == 0:
            votes_right += 1
        elif row[1] == 1 and row[3] == 1 and row[0] == 0 and row[2] == 0:
            votes_left += 1

    if votes_left >= 1 and votes_right >= 1:
        return False, False, True

    return votes_left > 0, votes_right > 0, False


def check_green(contours_grn, black_image, green_image):
    global contador_esquerda, contador_direita, contador_180
    black_around_sign = np.zeros((len(contours_grn), 5), dtype=np.int16)
    green_centers = []

    for i, contour in enumerate(contours_grn):
        if cv2.contourArea(contour) <= MIN_GREEN_AREA:
            green_centers.append(camera_x / 2)
            continue

        green_box = cv2.boxPoints(cv2.minAreaRect(contour))
        green_centers.append(float(np.mean(green_box[:, 0])))
        black_around_sign = check_black(black_around_sign, i, green_box, black_image)  # ← sem .copy()

    turn_left, turn_right, turn_180 = determine_turn_direction(black_around_sign, green_centers)
    """if not turn_180 and not turn_left and not turn_right:
        print("none")"""
    pixels_verde = cv2.countNonZero(green_image)
    #print(pixels_verde)
    if turn_180:
        contador_180 += 1
    else:
        contador_180 = 0

    if turn_left:
        contador_esquerda += 1
    else:
        contador_esquerda = 0

    if turn_right:
        contador_direita += 1
    else:
        contador_direita = 0

    if turn_180:
        return "GIRAR_180"
    elif turn_left and pixels_verde < 15500 and contador_esquerda > 2 and contador_direita < 1:
        return "ESQUERDA"
    elif turn_right  and pixels_verde < 15500 and contador_direita > 2 and contador_esquerda < 1:
        return "DIREITA"
    else:
        return "RETO"
def detectar_verde(frame):
    """Retorna acao verde ou None se nao encontrou nada relevante."""
    black_image = cv2.inRange(frame, black_min, black_max_bot)
    black_image[0:int(camera_y * 0.4), 0:camera_x] = cv2.inRange(
        frame, black_min, black_max_top)[0:int(camera_y * 0.4), 0:camera_x]
    black_image = cv2.erode(black_image,  kernal, iterations=2)
    black_image = cv2.dilate(black_image, kernal, iterations=5)
    black_image = cv2.erode(black_image,  kernal, iterations=3)

    hsv         = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_image = cv2.inRange(hsv, green_min, green_max)
    green_image = cv2.erode(green_image,  kernal, iterations=1)
    green_image = cv2.dilate(green_image, kernal, iterations=5)
    green_image = cv2.erode(green_image,  kernal, iterations=3)
    #print(cv2.countNonZero(green_image))
    contours_grn, _ = cv2.findContours(
        green_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours_grn) == 0:
        return None

    acao = check_green(contours_grn, black_image, green_image)
    return acao if acao != "RETO" else None

def achar_area(contours):
    area_preta = cv2.contourArea(max(contours, key=cv2.contourArea)) if contours else 0
    
    if distancia_esquerda() <= 15 and area_preta < AREA_MIN:
        print("achei a area")
        mover(-19, -19)
        time.sleep(0.09)
        parar()
        time.sleep(1000.0)

def area(): 
    print("estou na area")   
    print(distancia_esquerda())
    mover (-19,-19)
    time.sleep(0.09)
    parar()
    time.sleep(1000.0)

def girar_ate_linha(direcao, velocidade=60, tolerancia=40, timeout=5.0):
    inicio = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        cx, contours, roi, mask = visao(frame)

        #cv2.imshow("roi", roi)
        #cv2.imshow("mask", mask)
        #cv2.waitKey(1)

        w = roi.shape[1]
        centro = w // 2

        # Linha visível e centralizada — para
        if cx is not None and abs(cx - centro) < tolerancia:
            print("[girar_ate_linha] linha centralizada!")
            parar()
            break

        # Gira na direção pedida
        if direcao == "ESQUERDA":
            mover(-velocidade, velocidade)
        elif direcao == "DIREITA":
            mover(velocidade, -velocidade)
        else:
            mover(velocidade, velocidade)    
# =========================
# CONTROLE (SEU CÓDIGO)
# =========================

def limpar_camera(n=5):
    for _ in range(n):
        cap.read()



def controle(frame):
    global ultimo_erro, ultimo_tempo

    
    cx, contours, roi, mask = visao(frame)
    #print(distancia_esquerda())
    #maior = max(contours, key=cv2.contourArea)
    #print (abs(cv2.contourArea(maior)))

    em_gap = gap(contours)
    w = roi.shape[1]
    if em_gap:
        print("gap")
        cv2.putText(roi, "GAP", (10,30),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        while True:

            """if distancia_esquerda() < 15:
                mover(-19, -19)
                time.sleep(0.09)
                mover(-40, -40)
                time.sleep(0.8)
                if distancia_esquerda() > 25:
                    print(area)
                else :
                    print("nada so gap e continua ")
                break   """
            ret, frame = cap.read()
            if not ret:
                continue
            #print(distancia_esquerda())
            cx, contours, roi, mask = visao(frame)
            
            if not gap(contours) :
               
                print("saí")

                ultimo_erro = 0
                ultimo_tempo = time.time()

                parar()
                break   

            mover(VEL_GAP, VEL_GAP)
        return contours
        #print("area:", cv2.contourArea(max(contours, key=cv2.contourArea)) if contours else 0)
        #cv2.imshow("roi", roi)
        #cv2.imshow("mask", mask)
        #cv2.waitKey(1)

    # ===== CONTROLE NORMAL =====
    tempo_atual = time.time()
    dt = tempo_atual - ultimo_tempo
    if dt <= 0:
        dt = 0.0001
                   

    if cx is not None:
        centro = w // 2
        erro = ((cx - centro) / centro  *100)
    else:
        erro = ultimo_erro
    #print("ERRO", erro)
    if abs(erro) < DEADZONE:
        erro = 0
    
    derivada = (erro - ultimo_erro) / dt

    derivada = np.clip(derivada, -300, 300)

    ajuste = (Kp * erro) + (Kd * derivada)
    
    
    #print("AJUSTE", ajuste)       
    velL = VEL_BASE + ajuste
    velR = VEL_BASE - ajuste
    
    if angulo_y() < -10 :
        print("to pra cima ")
        while True:
            velocidade_base_ajustada = VEL_BASE + 30
            velL = velocidade_base_ajustada + ajuste
            velR = velocidade_base_ajustada - ajuste 
            velL = np.clip(velL, -30, 200)
            velR = np.clip(velR, -30, 200)
            mover(velL, velR)
            print("VEL ESQUERDA dentro do while" , velL)
            if angulo_y()>= -1:
                parar()
                time.sleep(2)
                break
        
        
    
    velL = np.clip(velL, VEL_MIN, VEL_MAX)
    velR = np.clip(velR, VEL_MIN, VEL_MAX)
    mover(velL, velR)
    print("VEL ESQUERDA fora do while" , velL)
    #print("VEL DIREITA",velR) 
    
    #mover(velL, velR)

    #cv2.imshow("roi", roi)
    #cv2.imshow("mask", mask)

    ultimo_erro = erro
    ultimo_tempo = tempo_atual
    return ajuste, contours

def desvio():
    if distancia_frente() <= 6:
        mover(-19,-19)
        time.sleep(0.09)
        guinada2('D', 87, 50)  
        mover(40, 40)
        time.sleep(0.15)
        mover(-19,-19)
        time.sleep(0.09)
        parar()
        time.sleep(0.5)
        while True:
            ret, frame = cap.read()
            if alinhar_linha(frame):
                print("Alinhado!")
                break
        '''mover(50, 50)
        time.sleep(0.1)
        mover(-19,-19)
        time.sleep(0.09)
        parar()
        time.sleep(0.5)
        while True:
            ret, frame = cap.read()
            if alinhar_linha(frame):
                print("Alinhado!")
                break'''
        mover(40, 40)
        time.sleep(0.8)
        mover(-19,-19)
        time.sleep(0.09)
        guinada2('E', 87, 40)
        mover(40,40)
        time.sleep(1.3)
        mover(-19,-19)
        time.sleep(0.09)
        parar()
        guinada2('E', 87, 40)
        mover(40, 40)
        time.sleep(0.7)
        mover(-19,-19)
        time.sleep(0.09)    
        guinada2('D', 87, 40)

def gangorra():
    
    """if angulo_y() >= 6:
        print("gangorra")
        while True:
            ret, frame = cap.read()
            ajuste, contours = controle(frame)
            velocidade_ajustada = (VEL_BASE + (angulo_y() * 3))
            ajuste2 = int(ajuste)
            velL = velocidade_ajustada + ajuste2
            velR = velocidade_ajustada - ajuste2
            mover(velL, velR)
            if angulo_y() <= 2:
                break  """      
                    

def verde(frame):
    acao_verde = detectar_verde(frame)
    if acao_verde:
        parar()
        #print ("acao de verde ", acao_verde)
        if acao_verde == "ESQUERDA":
            limpar_camera(10)
            guinada2('D', 9, 35)
            mover(50,50)
            time.sleep(0.21)
            mover(-19,-19)
            time.sleep(0.09)
            print("fui para frente e parei ")
            parar()
            print("virar esquerda ")
            guinada2('E', 90, 38)
            mover(-19,-19)
            time.sleep(0.09)

            parar()
            mover(50,50)
            time.sleep(0.20)
            mover(-19,-19)
            print("virei e parei ")
            limpar_camera(10)


        elif acao_verde == "DIREITA":
            limpar_camera(10)
            guinada2('E', 9, 35)
            mover(50,50)
            time.sleep(0.21)
            mover(-19,-19)
            time.sleep(0.09)
            print("fui para frente e parei ")
            parar()
            print("virar esquerda ")
            guinada2('D', 90, 38)
            mover(-19,-19)
            time.sleep(0.09)

            parar()
            mover(50,50)
            time.sleep(0.20)
            mover(-19,-19)
            print("virei e parei ")
            limpar_camera(10)

        elif acao_verde == "GIRAR_180":
            print("gira 180")
            mover(50,50)
            time.sleep(0.1)
            mover(-19,-19)
            time.sleep(0.09)
            parar()
            time.sleep(1.0)
            guinada2('E', 180, 35)
            mover(50,50)
            time.sleep(0.19)
            mover(-19,-19)
            time.sleep(0.09)
            """
            time.sleep(0.2)
            parar()"""
            limpar_camera(10)

def guinada2(LADO, GRAUS, VELOCIDADE):
    resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS - 10)
    if LADO == 'D':
        while True:
            print(angulo_z())
            mover(VELOCIDADE, -VELOCIDADE)
            if angulo_z() <= -Graus:   # ← era >= , trocado
                print("PAROU")
                mover(-20, 20)
                time.sleep(0.09)
                break
    else:
        while True:
            print(angulo_z())
            mover(-VELOCIDADE, VELOCIDADE)
            if angulo_z() >= Graus:    # ← era <= -Graus, trocado
                print("PAROU")
                mover(20, -20)
                time.sleep(0.09)
                break

def detectar_fita_vermelha(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0,   100, 100]), np.array([10,  255, 255])),
        cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
    )

    h, w = mask.shape
    for y in range(0, h, 5):
        if cv2.countNonZero(mask[y, :]) > w * 0.6:
            print("[VERMELHO] Fita detectada — pausando 6s")
            parar()
            time.sleep(6)
            limpar_camera(10)
            return True
    return False
# =========================
# LOOP
# =========================

"""time.sleep(1.0)"""
#guinada2('D', 90, 35)
parar()
#time.sleep(3.0)
try:
    while True:
        #distancia_F = ler_ultrassonico()
        # BOTÃO
        estado = GPIO.gpio_read(h, BUTTON)
        
        #print("outro")
        if estado == 1:
            parar()
            print("parado")
            resetar_giroscopio()
            #receberdado("R")
            continue
        
        ret, frame = cap.read()
        if detectar_fita_vermelha(frame):
            continue
        else:
            '''alinhar_linha(frame)
            cv2.imshow("ROI", visao(frame)[2])'''
            contours = controle(frame)
            verde(frame)
            #gangorra()
            #desvio() 
            #print(angulo_y())
            #if contours: 
                #achar_area(contours)
            #print(distancia_frente())
            
            #time.sleep(2.0)
            #print(angulo_z())
        if cv2.waitKey(1) == 27:
            break
        if not ret and contours is None:
            continue

       

except KeyboardInterrupt:
    print("Finalizado")

finally:
    parar()
    GPIO.gpiochip_close(h)
    cap.release()
    cv2.destroyAllWindows()