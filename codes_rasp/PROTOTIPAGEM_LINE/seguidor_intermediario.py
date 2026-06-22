import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import serial
import serial.tools.list_ports
GPIO.cleanup()
# =========================
# CONFIG (NÃO ALTERADO)
# =========================
Kp = 4 #4
Kd = 0.5 #0.5
VEL_BASE = 30 #30
VEL_MAX  = 50 #50
VEL_MIN  = -VEL_MAX #-50
VEL_GAP = 30
LIMIAR   = 80
DEADZONE = 4 #4
AREA_MIN = 6000
girando = False

# =========================
# ESTADO
# =========================
tempo_gap = 0
ultimo_erro = 0
ultimo_tempo = time.time()

# =========================
# GPIO (NÃO ALTERADO)
# =========================
IN1, IN2, ENA = 17, 27, 12
IN3, IN4, ENB = 22, 23, 13
BUTTON = 24

green_min = np.array([40, 50, 45])
green_max = np.array([85, 255, 255])

black_min     = np.array([0, 0, 0])
black_max_top = np.array([90, 90, 90])
black_max_bot = np.array([135, 135, 135])

MIN_GREEN_AREA = 2500

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
                s = serial.Serial(p.device, 115200, timeout=0.0)
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

time.sleep(2)
ser.reset_input_buffer()
ser.write(b"START\n")

print("Handshake enviado pro Arduino")

# =========================
# GPIO INIT
# =========================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwmA = GPIO.PWM(ENA, 1000)
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0)
pwmB.start(0)


# =========================
# FUNÇOES DE COMUNICACAO COM NANO (NÃO ALTERADO)
# =========================

def receberdado(msg):
    ser.write((msg + '\n').encode())
    dado = receber()
    return dado 

# RECEBER
def receber():
    while True:
        if ser.in_waiting:
            linha = ser.readline().decode().strip()
            if linha:
                return linha
    

# =========================
# MOTORES (NÃO ALTERADO)
# =========================
def aplicar_motor(in1, in2, pwm, vel):
    if vel > 0:
        GPIO.output(in1, 1)
        GPIO.output(in2, 0)
    elif vel < 0:
        GPIO.output(in1, 0)
        GPIO.output(in2, 1)
    else:
        GPIO.output(in1, 0)
        GPIO.output(in2, 0)

    pwm.ChangeDutyCycle(min(abs(vel), VEL_MAX))

def mover(velL, velR):
    aplicar_motor(IN1, IN2, pwmA, velL)
    aplicar_motor(IN3, IN4, pwmB, velR)

def parar():
    mover(0, 0)

# =========================
# CAMERA
# =========================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
time.sleep(1.0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

camera_x = 320
camera_y = 240

cap.set(3, camera_x)
cap.set(4, camera_y)

# =========================
# ESTADO
# =========================

rodando = False
ultimo_estado = 1
kernal = np.ones((5, 5), np.uint8)
# =========================
# VISÃO (TUDO JUNTO)
# =========================
def visao(frame):
    h, w = frame.shape[:2]
    roi = frame[int(h*0.1):h, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, LIMIAR, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

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


def check_black(black_around_sign, i, green_box, black_image):
    green_box = green_box[green_box[:, 1].argsort()]
    marker_height = green_box[-1][1] - green_box[0][1]
    black_around_sign[i, 4] = int(green_box[2][1])

    # Baixo
    r = black_image[
        int(green_box[2][1]) : min(int(green_box[2][1] + int(marker_height * 0.8)), camera_y),
        min(int(green_box[2][0]), int(green_box[3][0])) : max(int(green_box[2][0]), int(green_box[3][0]))
    ]
    if r.size > 0 and np.mean(r) > 125:
        black_around_sign[i, 0] = 1

    # Cima
    r = black_image[
        max(int(green_box[1][1] - int(marker_height * 0.8)), 0) : int(green_box[1][1]),
        min(max(int(green_box[0][0]), 0), max(int(green_box[1][0]), 0)) : max(max(int(green_box[0][0]), 0), max(int(green_box[1][0]), 0))
    ]
    if r.size > 0 and np.mean(r) > 125:
        black_around_sign[i, 1] = 1

    green_box = green_box[green_box[:, 0].argsort()]

    # Esquerda
    r = black_image[
        min(int(green_box[0][1]), int(green_box[1][1])) : max(int(green_box[0][1]), int(green_box[1][1])),
        max(int(green_box[1][0] - int(marker_height * 0.8)), 0) : int(green_box[1][0])
    ]
    if r.size > 0 and np.mean(r) > 125:
        black_around_sign[i, 2] = 1

    # Direita
    r = black_image[
        min(int(green_box[2][1]), int(green_box[3][1])) : max(int(green_box[2][1]), int(green_box[3][1])),
        int(green_box[2][0]) : min(int(green_box[2][0] + int(marker_height * 0.8)), camera_x)
    ]
    if r.size > 0 and np.mean(r) > 125:
        black_around_sign[i, 3] = 1

    return black_around_sign


def determine_turn_direction(black_around_sign):
    turn_left  = False
    turn_right = False
    for i in black_around_sign:
        if np.sum(i[:4]) == 2:
            if i[1] == 1 and i[2] == 1:
                turn_right = True
            elif i[1] == 1 and i[3] == 1:
                turn_left = True
    return turn_left, turn_right


def check_green(contours_grn, black_image):
    black_around_sign = np.zeros((len(contours_grn), 5), dtype=np.int16)

    for i, contour in enumerate(contours_grn):
        if cv2.contourArea(contour) <= MIN_GREEN_AREA:
            continue
        green_box = cv2.boxPoints(cv2.minAreaRect(contour))
        black_around_sign = check_black(black_around_sign, i, green_box, black_image.copy())

    turn_left, turn_right = determine_turn_direction(black_around_sign)

    if turn_left and turn_right:
        return "GIRAR_180"
    elif turn_left:
        return "ESQUERDA"
    elif turn_right:
        return "DIREITA"
    else:
        return "RETO"


def detectar_verde(frame):

    """Retorna:
       ESQUERDA
       DIREITA
       GIRAR_180
       None
    """

    # =========================================
    # ROI INFERIOR
    # =========================================
    roi = frame[int(camera_y * 0.35):camera_y, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # =========================================
    # MÁSCARA VERDE
    # =========================================
    green_image = cv2.inRange(hsv, green_min, green_max)

    green_image = cv2.erode(green_image, kernal, iterations=1)
    green_image = cv2.dilate(green_image, kernal, iterations=4)
    green_image = cv2.erode(green_image, kernal, iterations=2)

    h, w = green_image.shape

    # =========================================
    # ZONAS
    # =========================================
    LEFT_END = int(w * 0.35)
    RIGHT_START = int(w * 0.65)

    left_mask = green_image[:, 0:LEFT_END]
    right_mask = green_image[:, RIGHT_START:w]

    # =========================================
    # PIXELS VERDES
    # =========================================
    pixels_left = cv2.countNonZero(left_mask)
    pixels_right = cv2.countNonZero(right_mask)

    # DEBUG
    cv2.imshow("mask_verde", green_image)

    print("LEFT:", pixels_left, "RIGHT:", pixels_right)

    # =========================================
    # LIMIAR
    # =========================================
    MIN_GREEN = 1800

    left_detect = pixels_left > MIN_GREEN
    right_detect = pixels_right > MIN_GREEN

    # =========================================
    # DUPLO VERDE
    MIN_GREEN = 2500
    MIN_DOUBLE = 1200

    # =========================================
    # DUPLO VERDE
    # =========================================
    if pixels_left > MIN_DOUBLE and pixels_right > MIN_DOUBLE:
        return "GIRAR_180"

    # =========================================
    # ESQUERDA
    # =========================================
    elif pixels_left > MIN_GREEN:
        return "ESQUERDA"

    # =========================================
    # DIREITA
    # =========================================
    elif pixels_right > MIN_GREEN:
        return "DIREITA"
    return None
def guinada(direcao, graus, velocidade=40, tolerancia=2.0):
    ang_atual = float(receberdado("Z"))
    
    alvo = ang_atual + graus if direcao == "DIREITA" else ang_atual - graus
    while alvo > 180:  alvo -= 360
    while alvo < -180: alvo += 360

    while True:
        ang = float(receberdado("Z"))
        erro = alvo - ang
        while erro > 180:  erro -= 360
        while erro < -180: erro += 360

        if abs(erro) <= tolerancia:
            parar()
            break

        vel = np.clip(abs(erro) * 1.2, 35, velocidade)

        if erro > 0:
            mover(vel, -vel)
        else:
            mover(-vel, vel)

# =========================
# CONTROLE (SEU CÓDIGO)
# =========================

def limpar_camera(n=5):
    for _ in range(n):
        cap.read()
def controle(frame):
    global ultimo_erro, ultimo_tempo

    
    
    cx, contours, roi, mask = visao(frame)

    maior = max(contours, key=cv2.contourArea)
    #print (abs(cv2.contourArea(maior)))

    em_gap = gap(contours)
    w = roi.shape[1]
    if em_gap:
        print("gap")
        cv2.putText(roi, "GAP", (10,30),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            cx, contours, roi, mask = visao(frame)

            if not gap(contours):
                print("saí")

                ultimo_erro = 0
                ultimo_tempo = time.time()

                parar()
                break

            mover(VEL_GAP, VEL_GAP)
            #print("area:", cv2.contourArea(max(contours, key=cv2.contourArea)) if contours else 0)
            cv2.imshow("roi", roi)
            cv2.imshow("mask", mask)
            cv2.waitKey(1)
        return
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
    velL = np.clip(velL, VEL_MIN, VEL_MAX)
    velR = np.clip(velR, VEL_MIN, VEL_MAX)
    #print("VEL ESQUERDA" , velL)
    #print("VEL DIREITA",velR) 
    mover(velL, velR)

    cv2.imshow("roi", roi)
    cv2.imshow("mask", mask)

    ultimo_erro = erro
    ultimo_tempo = tempo_atual

# =========================
# LOOP
# =========================
try:
    while True:

        # BOTÃO
        """estado = GPIO.input(BUTTON)
        if estado == 0 and ultimo_estado == 1:
            rodando = not rodando
            print("RODANDO:", rodando)
        ultimo_estado = estado

        if not rodando:
            parar()
            continue"""

        ret, frame = cap.read()
        if not ret:
            parar()
            continue

        # ── 1. Verde (prioridade máxima) ──────────────────────
        acao_verde = detectar_verde(frame)
        if acao_verde and not girando :
            girando = True 
            
            print(f"[VERDE] {acao_verde}")
            
            if acao_verde == "ESQUERDA":
                acao_verde = detectar_verde(frame)
                print("virar_esquerda")
                mover(60,60)
                time.sleep(0.17)
                parar()
                time.sleep(2.0)
                guinada("ESQUERDA", 87, 90, 3)
                print("virei 90")
                limpar_camera(10)
                mover(50,50)
                time.sleep(0.2)
            elif acao_verde == "DIREITA":
                print("virar_DIREITa")
                mover(60,60)
                time.sleep(0.17)
                parar()
                time.sleep(2.0)
                guinada("DIREIA", 87, 90, 6)
                print("virei 90")
                limpar_camera(10)
                mover(50,50)
                time.sleep(0.3)
            elif acao_verde == "GIRAR_180":
                print("gira 180")
                mover(50,50)
                time.sleep(0.1)
                parar()
                guinada("DIREITA", 170, 90, 2)
                mover(50,50)
                time.sleep(0.2)
                parar()
                limpar_camera(10)
            girando = False 
            # após a ação, volta pro loop normalmente

        # ── 3. Seguidor normal ────────────────────────────────
        else:
        
            controle(frame)

        if cv2.waitKey(1) == 27:
            break

       

except KeyboardInterrupt:
    print("Finalizado")

finally:
    parar()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()