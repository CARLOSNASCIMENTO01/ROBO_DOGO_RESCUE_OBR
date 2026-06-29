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
Kp = 1.6 #2.7
Kd = 0.7 #0.9
tempo_sem_linha = 0 
VEL_BASE = 27 # 24
VEL_MAX  = 50 #40
VEL_MIN  = -VEL_MAX #-50
VEL_GAP = 27
LIMIAR   = 80
DEADZONE = 4#4
AREA_MIN = 11000 #6000
girando = False

em_rampa = False

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
# DEBUG — mude para True pra ativar janelas e prints de verde
# =========================
DEBUG_VERDE = True

# =========================
# DEBUG SEM AÇÃO — True: detecta e imprime verde mas NÃO executa giros
# Útil pra calibrar detecção com o robô seguindo linha normalmente
# =========================
DEBUG_SEM_ACAO = False
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

def angulo_linha(contours):
    if not contours:
        return None
    
    # filtra contornos grandes o suficiente
    validos = [c for c in contours if cv2.contourArea(c) >= AREA_MIN]
    
    if not validos:
        return None
    
    melhor = None
    menor_angulo = 9999

    for c in validos:
        [vx, vy, x0, y0] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
        angulo = math.degrees(math.atan2(float(vy), float(vx)))

        if angulo > 90:
            angulo -= 180
        elif angulo < -90:
            angulo += 180

        angulo = int(angulo - 90)

        if abs(angulo) < abs(menor_angulo):
            menor_angulo = angulo
            melhor = c

    return menor_angulo    
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
        black_around_sign = check_black(black_around_sign, i, green_box, black_image)

    turn_left, turn_right, turn_180 = determine_turn_direction(black_around_sign, green_centers)

    pixels_verde = cv2.countNonZero(green_image)
    print(pixels_verde)
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
    elif turn_left and pixels_verde < 15500: #and pixels_verde < 15500 and contador_esquerda > 2 and contador_direita < 1:
        return "ESQUERDA"
    elif turn_right and  pixels_verde < 15500 :#and pixels_verde < 15500 and contador_direita > 2 and contador_esquerda < 1:
        return "DIREITA"
    else:
        return "RETO"


# =========================
# DETECTAR VERDE
# MUDANÇA: recebe roi (já cortado por visao()) em vez do frame inteiro
# Morfologia reduzida: era erode(2)+dilate(5)+erode(3), agora dilate(3)+erode(2)
# =========================
def detectar_verde(roi):
    """Retorna acao verde ou None. Recebe roi já calculado por visao()."""
    roi_h, roi_w = roi.shape[:2]
    corte_top = int(roi_h * 0.4)

    # --- Preto ---
    black_image = cv2.inRange(roi, black_min, black_max_bot)
    black_image[:corte_top, :] = cv2.inRange(
        roi[:corte_top, :], black_min, black_max_top)
    # morfologia reduzida
    black_image = cv2.dilate(black_image, kernal, iterations=3)
    black_image = cv2.erode(black_image,  kernal, iterations=2)

    # --- Verde ---
    hsv         = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    green_image = cv2.inRange(hsv, green_min, green_max)
    # morfologia reduzida
    green_image = cv2.dilate(green_image, kernal, iterations=3)
    green_image = cv2.erode(green_image,  kernal, iterations=2)

    contours_grn, _ = cv2.findContours(
        green_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # =======================
    # DEBUG VERDE — ative com DEBUG_VERDE = True no topo do arquivo
    # Mostra: roi original | mask verde | mask preto | contornos + ação
    # =======================
    # if DEBUG_VERDE:
    #     debug_roi   = roi.copy()
    #     debug_verde = cv2.cvtColor(green_image, cv2.COLOR_GRAY2BGR)
    #     debug_preto = cv2.cvtColor(black_image, cv2.COLOR_GRAY2BGR)
    #
    #     pixels_verde = cv2.countNonZero(green_image)
    #     n_contornos  = len(contours_grn)
    #
    #     for c in contours_grn:
    #         if cv2.contourArea(c) > MIN_GREEN_AREA:
    #             cv2.drawContours(debug_roi, [c], -1, (0, 255, 0), 1)
    #             box = cv2.boxPoints(cv2.minAreaRect(c))
    #             box = np.int0(box)
    #             cv2.drawContours(debug_roi, [box], -1, (0, 165, 255), 1)
    #
    #     label = f"px:{pixels_verde} cnt:{n_contornos}"
    #     cv2.putText(debug_roi, label, (2, 10),
    #                 cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
    #
    #     linha = np.hstack([debug_roi, debug_verde, debug_preto])
    #     cv2.imshow("DEBUG VERDE | roi | verde | preto", linha)
    #     cv2.waitKey(1)
    #
    #     print(f"[DEBUG VERDE] pixels={pixels_verde} | contornos={n_contornos} "
    #           f"| esq={contador_esquerda} dir={contador_direita} 180={contador_180}")
    # =======================

    if len(contours_grn) == 0:
        return None

    acao = check_green(contours_grn, black_image, green_image)

    # =======================
    # DEBUG AÇÃO — imprime decisão final do verde
    # =======================
    # if DEBUG_VERDE and acao and acao != "RETO":
    #     print(f"[DEBUG VERDE] AÇÃO DETECTADA: {acao}")
    # =======================

    return acao if acao != "RETO" else None


def achar_area(contours):
    area_preta = cv2.contourArea(max(contours, key=cv2.contourArea)) if contours else 0
    
    if distancia_esquerda() <= 15 and area_preta < AREA_MIN:
        print("achei a area")
        mover(-19, -19)
        time.sleep(0.09)
        parar()
        time.sleep(1000.0)

def area(contours): 
    print("estou na area")   
    print(distancia_esquerda())
    mover (-19,-19)
    time.sleep(0.09)

    maior = max(contours, key=cv2.contourArea)

    while True:
        mover(25,25)
        if distancia_frente() <= 6 or len(contours) == 1 and cv2.contourArea(maior) > AREA_MIN:
            mover(-10,-10)
            time.sleep(0.09)
            if distancia_frente() <= 6:
                guinada2("E", 45, 40)
            if len(contours) == 1 and cv2.contourArea(maior) > AREA_MIN:
                mover(-10,-10)
                time.sleep(0.09)

def girar_ate_linha(direcao, velocidade=60, tolerancia=40, timeout=5.0):
    inicio = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        cx, contours, roi, mask = visao(frame)

        w = roi.shape[1]
        centro = w // 2

        if cx is not None and abs(cx - centro) < tolerancia:
            print("[girar_ate_linha] linha centralizada!")
            parar()
            break

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


def controle(cx, contours, roi):
    """
    MUDANÇA: não chama visao() internamente.
    Recebe cx, contours, roi já calculados no loop principal.
    """
    global ultimo_erro, ultimo_tempo, em_rampa

    em_gap = gap(contours)
    w = roi.shape[1]

    tempo_atual = time.time()
    dt = tempo_atual - ultimo_tempo
    if dt <= 0:
        dt = 0.0001

    if cx is not None:
        centro = w // 2
        erro = ((cx - centro) / centro * 100)
    else:
        erro = ultimo_erro

    if abs(erro) < DEADZONE:
        erro = 0
    if gap_debug(contours):
        erro = 0

    derivada = (erro - ultimo_erro) / dt
    derivada = np.clip(derivada, -300, 300)

    ajuste = (Kp * erro) + (Kd * derivada)

    if angulo_y() <= -6:
        velocidade_base = 60
        velL = velocidade_base + ajuste
        velR = velocidade_base - ajuste
        velL = np.clip(velL, 20, 100)
        velR = np.clip(velR, 20, 100)
    else:
        velocidade_base = VEL_BASE
        velL = velocidade_base + ajuste
        velR = velocidade_base - ajuste
        velL = np.clip(velL, VEL_MIN, VEL_MAX)
        velR = np.clip(velR, VEL_MIN, VEL_MAX)

    mover(velL, velR)

    ultimo_erro = erro
    ultimo_tempo = tempo_atual
    return ajuste, contours

def desvio():
    if distancia_frente() <= 6:
        mover(-30,-30)
        time.sleep(0.2)
        mover(-10,-10)
        time.sleep(0.1)
        mover(-10,-10)
        time.sleep(0.09)
        guinada2('D', 55, 35) 
        mover(-10,-10)
        time.sleep(0.2)
        while True :
            mover(-30,30)
            print(distancia_esquerda())
            if distancia_frente() < 15:
                mover(15,-15)
                time.sleep(0.3)
                break  
        guinada2("D",70,40)
        mover(35,35)
        time.sleep(0.6)
        mover(-10,-10)
        time.sleep(0.1)
        guinada2("E",90,35)
        while True :
            mover(30,30)
            print(distancia_esquerda())
            if distancia_esquerda() < 30:
                mover(-10,-10)
                time.sleep(0.3)
                break 
        while True :
            mover(30,30)
            print(distancia_esquerda())
            if distancia_esquerda() > 15:
                mover(-10,-10)
                time.sleep(0.3)
                break
            
        mover(40,40)
        time.sleep(0.3)
        mover(-10,-10)
        time.sleep(0.09)
        guinada2("E", 90, 40)
        mover(40,40)
        time.sleep(0.5)
        mover(-10,-10)
        time.sleep(0.09)
        guinada2("D", 87, 40)
        mover(-30,-30)
        time.sleep(0.1)



def verde(roi):
    acao_verde = detectar_verde(roi)
    if acao_verde:
    
        if DEBUG_SEM_ACAO:
            print(f"[DEBUG SEM AÇÃO] Verde detectado: {acao_verde} "
                  f"| esq={contador_esquerda} dir={contador_direita} 180={contador_180}")
            return

        print(f"[VERDE] Primeira detecção: {acao_verde} — avançando 80ms")
        inicio_confirmacao = time.time()
        acao_confirmada = acao_verde
 
        while time.time() - inicio_confirmacao < 0.02:
            ret, frame_conf = cap.read()
            if not ret:
                continue
            cx_c, contours_c, roi_c, _ = visao(frame_conf)
            controle(cx_c, contours_c, roi_c)   
            nova_acao = detectar_verde(roi_c)
            if nova_acao:
                acao_confirmada = nova_acao      
      
 
        acao_verde = acao_confirmada
    
        parar()
       
        if acao_verde == "ESQUERDA":
            mover(-10,-10)
            time.sleep(0.1)
            mover(30,30)
            time.sleep(0.15)
            print("fui para frente e parei ")
            parar()
            print("virar esquerda ")
            guinada2('E', 80, 40)
            inicio = time.time()
            while time.time() - inicio < 1.0:
                ret, frame = cap.read()
                print("segui linha")
                if not ret:
                    continue
                cx, contours, roi_loop, mask = visao(frame)
                controle(cx, contours, roi_loop)

        elif acao_verde == "DIREITA":
            limpar_camera(10)
            mover(-10,-10)
            time.sleep(0.1)
            mover(30,30)
            time.sleep(0.15)
            print("fui para frente e parei ")
            parar()
            print("virar direita ")
            guinada2('D', 80, 40)
            inicio = time.time()
            while time.time() - inicio < 1.0:
                ret, frame = cap.read()
                print("segui linha")
                if not ret:
                    continue
                cx, contours, roi_loop, mask = visao(frame)
                controle(cx, contours, roi_loop)

        elif acao_verde == "GIRAR_180":
            print("gira 180")
            mover(-12,-12)
            time.sleep(0.09)
            guinada2('E', 180, 40)
            mover(30,30)
            time.sleep(0.19)
            mover(-10,-10)
            time.sleep(0.2)
            parar()
            limpar_camera(10)

def guinada2(LADO, GRAUS, VELOCIDADE):
    resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS - 13)
    if LADO == 'D':
        while True:
            print(angulo_z())
            mover(VELOCIDADE, -VELOCIDADE)
            if angulo_z() <= -Graus:
                print("PAROU")
                mover(-10, 10)
                time.sleep(0.05)
                break
    else:
        while True:
            print(angulo_z())
            mover(-VELOCIDADE, VELOCIDADE)
            if angulo_z() >= Graus:
                print("PAROU")
                mover(10, -10)
                time.sleep(0.05)
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
def andar_reto(velocidade=30, segundos=1.0):

    resetar_giroscopio()

    Kp = -2
    Ki = -0.2
    Kd = -0.4

    erro_anterior = 0
    integral = 0

    ultimo_tempo = time.time()
    inicio = ultimo_tempo

    while time.time() - inicio < segundos:

        agora = time.time()
        dt = agora - ultimo_tempo

        if dt <= 0:
            dt = 0.001

        erro = angulo_z()

        integral += erro * dt
        integral = np.clip(integral, -30, 30)

        derivada = (erro - erro_anterior) / dt

        ajuste = (
            Kp * erro +
            Ki * integral +
            Kd * derivada
        )

        velL = velocidade - ajuste
        velR = velocidade + ajuste

        mover(velL, velR)

        erro_anterior = erro
        ultimo_tempo = agora

        time.sleep(0.01)

    parar()

def gap_debug(contours):
    if len(contours) == 0:
        return True

    maior = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(maior)

    if area < AREA_MIN:
        box = cv2.boxPoints(cv2.minAreaRect(maior))
        box = box[box[:, 1].argsort()]

        p1, p2 = box[0], box[1]
        vector = p1 - p2
        norma = np.linalg.norm(vector)

        if norma > 0:
            angulo = math.degrees(math.acos(
                np.clip(np.dot(vector, [1, 0]) / norma, -1, 1)
            ))
            angulo = angulo if p1[0] < p2[0] else -angulo
            if angulo == 180:
                angulo = 0
        else:
            angulo = 0

        return True

    print(f"[GAP] Não tem gap | Área: {area:.0f}")
    return False


# =========================
# DETECÇÃO DE TRAVAMENTO
# Compara frame atual com frame de N segundos atrás (não o anterior)
# =========================
_trav_frame_ref  = None
_trav_tempo_ref  = time.time()
LIMIAR_DIFF      = 8      # diff média mínima pra considerar movimento (0-255)
TEMPO_TRAV       = 4.0    # segundos sem movimento pra declarar travamento
INTERVALO_REF    = 2.0    # atualiza frame de referência a cada N segundos

def detectar_travamento(roi):
    global _trav_frame_ref, _trav_tempo_ref

    gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    agora = time.time()

    # ainda não tem referência — guarda e sai
    if _trav_frame_ref is None:
        _trav_frame_ref = gray.copy()
        _trav_tempo_ref = agora
        return False

    movimento = np.mean(cv2.absdiff(gray, _trav_frame_ref))

    # atualiza referência a cada INTERVALO_REF segundos
    if agora - _trav_tempo_ref >= INTERVALO_REF:
        if movimento >= LIMIAR_DIFF:
            # havia movimento no intervalo — reseta tudo
            _trav_frame_ref = gray.copy()
            _trav_tempo_ref = agora
            return False
        else:
            # sem movimento no intervalo — acumula mas não atualiza ref
            tempo_parado = agora - _trav_tempo_ref
            print(f"[TRAV] diff={movimento:.1f} | parado há {tempo_parado:.1f}s")
            if tempo_parado >= TEMPO_TRAV:
                print(f"[TRAVAMENTO] Robô travado!")
                _trav_frame_ref = None  # reseta pra próxima detecção
                _trav_tempo_ref = agora
                return True

    return False
parar()

try:
    while True:
        estado = GPIO.gpio_read(h, BUTTON)
        if estado == 1:
            parar()
            resetar_giroscopio()
            continue

        ret, frame = cap.read()
        if not ret:
            continue

        if detectar_fita_vermelha(frame):
            continue

        # =========================
        # MUDANÇA PRINCIPAL: visao() uma única vez por frame
        # verde() ANTES de controle() para reagir mais cedo
        # =========================
        cx, contours, roi, mask = visao(frame)
        if detectar_travamento(roi):
            print("[TRAVAMENTO] Tentando se soltar...")
            while True:
                mover(100, 100)
                time.sleep(0.2)
                
                ret, frame_trav = cap.read()
                if ret:
                    _, _, roi_trav, _ = visao(frame_trav)
                    if not detectar_travamento(roi_trav):
                        print("[TRAVAMENTO] Solto!")
                        parar()
                        ultimo_erro  = 0          # ← reseta estado do controle
                        ultimo_tempo = time.time() # ← evita dt gigante na volta
                break

        verde(roi)           # ← verde primeiro, recebe roi já pronto
        ajuste, contours = controle(cx, contours, roi)  # ← aproveita visao() já feito
        desvio()

        if gap_debug(contours) and distancia_esquerda() < 10:
            inicio_gap = time.time()
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue
                
                cx, contours, roi, mask = visao(frame)
                
                if not gap_debug(contours):
                    print("[GAP] Saiu do gap, voltando à linha")
                    break
                
                if time.time() - inicio_gap > 1:
                    
                    area(contours)
                    break
                
                mover(VEL_GAP, VEL_GAP)
                        
        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    print("Finalizado")

finally:
    parar()
    GPIO.gpiochip_close(h)
    cap.release()
    cv2.destroyAllWindows()