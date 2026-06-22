import time
import cv2
import numpy as np

# ============================================
# GPIO / MOTOR
# ============================================
try:
    import RPi.GPIO as GPIO
    ON_RPI = True
except:
    ON_RPI = False

    class DummyGPIO:
        BCM = "BCM"
        OUT = "OUT"
        IN = "IN"
        PUD_UP = "PUD_UP"
        def setmode(self, x): pass
        def setwarnings(self, x): pass
        def setup(self, x, y, pull_up_down=None): pass
        def input(self, x): return 1
        def output(self, x, y): pass
        def PWM(self, pin, freq):
            class PWM:
                def start(self, x): pass
                def ChangeDutyCycle(self, x): pass
                def stop(self): pass
            return PWM()
        def cleanup(self): pass

    GPIO = DummyGPIO()

# ============================================
# PINOS
# ============================================
IN1, IN2, ENA = 17, 27, 12
IN3, IN4, ENB = 22, 23, 13
BUTTON = 24

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm_left  = GPIO.PWM(ENA, 1000)
pwm_right = GPIO.PWM(ENB, 1000)
pwm_left.start(0)
pwm_right.start(0)

# ============================================
# CONFIG
# ============================================
CAM_W_ORIG = 320
CAM_H_ORIG = 240

# após ROTATE_180 dimensões não mudam
CAM_W = 160
CAM_H = 120

VEL_RETA  = 48
VEL_GIRO  = 48
DEAD_ZONE = 20
AREA_MIN  = 500

# ============================================
# ESTADO
# ============================================
rodando        = False
ultimo_botao   = 1
erro_suavizado = 0.0
x_last = CAM_W // 2
y_last = CAM_H // 2

# ============================================
# MOTOR
# ============================================
def aplicar_motor(in1, in2, pwm, vel):
    if vel > 0:
        GPIO.output(in1, 1); GPIO.output(in2, 0)
    elif vel < 0:
        GPIO.output(in1, 0); GPIO.output(in2, 1)
    else:
        GPIO.output(in1, 0); GPIO.output(in2, 0)
    pwm.ChangeDutyCycle(min(abs(vel), 100))

def mover(velL, velR):
    aplicar_motor(IN1, IN2, pwm_left,  velL)
    aplicar_motor(IN3, IN4, pwm_right, velR)

def parar():
    mover(0, 0)

# ============================================
# CAMERA
# ============================================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W_ORIG)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H_ORIG)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
time.sleep(1)

# ============================================
# VISÃO
# ============================================
def processar_imagem(frame):
    global x_last, y_last

    frame = cv2.rotate(frame, cv2.ROTATE_180)
    h, w = frame.shape[:2]

    # ── MÁSCARA PRETA ────────────────────────────────────────────────────
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_black = np.array([0,   0,   0])
    upper_black = np.array([180, 255, 75])

    mask = cv2.inRange(hsv, lower_black, upper_black)

    # ── REMOVE REFLEXOS DO LED ────────────────────────────────────────────
    brilho       = hsv[:, :, 2]
    mask_reflexo = (brilho > 200).astype(np.uint8) * 255
    mask         = cv2.bitwise_and(mask, cv2.bitwise_not(mask_reflexo))

    # ── MORPHOLOGY ────────────────────────────────────────────────────────
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode( mask, kernel, iterations=3)
    mask = cv2.dilate(mask, kernel, iterations=5)
    mask = cv2.erode( mask, kernel, iterations=2)

    # ── CONTORNOS ─────────────────────────────────────────────────────────
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if not contours:
        return None, mask, frame

    contours = [c for c in contours if cv2.contourArea(c) > AREA_MIN]

    if not contours:
        return None, mask, frame

    # ── MELHOR CONTORNO ───────────────────────────────────────────────────
    melhor     = None
    menor_dist = 999999

    for c in contours:
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        dist = abs(cx - x_last) + abs(cy - y_last)
        if dist < menor_dist:
            menor_dist = dist
            melhor = c

    if melhor is None:
        return None, mask, frame

    # ── CENTRO ────────────────────────────────────────────────────────────
    M  = cv2.moments(melhor)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    x_last = cx
    y_last = cy

    # ── ÂNGULO por desvio do centro — estável, sem fitLine ────────────────
    # erro_x: quanto o centro da linha desviou do centro da imagem
    # normalizado para -90..+90
    centro = w // 2
    erro_x = cx - centro                            # px: neg=esquerda, pos=direita
    angle  = float(erro_x) / (w / 2) * 90.0
    angle = -angle        # escala para graus
    angle  = float(np.clip(angle, -90.0, 90.0))

    # ── VISUAL ────────────────────────────────────────────────────────────
    cv2.drawContours(frame, [melhor], -1, (255, 0, 0), 2)
    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

    # linha do centro da imagem
    cv2.line(frame, (centro, 0), (centro, h), (255, 255, 0), 1)

    # linha do centro da linha detectada
    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 2)

    cv2.putText(frame, f"ANGULO: {angle:+.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame, f"CX: {cx}  CENTRO: {centro}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    return angle, mask, frame

# ============================================
# CONTROLE BINÁRIO
# ============================================
def controle(angle):
    global erro_suavizado

    if angle is None:
        parar()
        return

    erro_suavizado = erro_suavizado * 0.7 + angle * 0.3
    erro = erro_suavizado

    if -DEAD_ZONE <= erro <= DEAD_ZONE:
        velL = VEL_RETA;  velR = VEL_RETA;  estado = "RETO"
    elif erro > DEAD_ZONE:
        velL =  VEL_GIRO; velR = -VEL_GIRO; estado = "GIRA DIREITA"
    else:
        velL = -VEL_GIRO; velR =  VEL_GIRO; estado = "GIRA ESQUERDA"

    mover(velL, velR)
    print(f"ANGULO: {erro:+.1f}° | {estado} | L={velL} R={velR}")

# ============================================
# LOOP
# ============================================
try:
    while True:

        estado_btn = GPIO.input(BUTTON)
        if estado_btn == 0 and ultimo_botao == 1:
            rodando = not rodando
            print("RODANDO:", rodando)
            time.sleep(0.25)
        ultimo_botao = estado_btn

        ret, frame = cap.read()
        if not ret:
            continue

        angle, mask, vis = processar_imagem(frame)

        if rodando:
            controle(angle)
        else:
            parar()

        cv2.imshow("MASK", mask)
        cv2.imshow("LINE", vis)

        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    print("FINALIZADO")

finally:
    parar()
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()