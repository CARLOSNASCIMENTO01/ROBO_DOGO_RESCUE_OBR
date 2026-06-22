from buildhat import Motor
import time 
motorE = Motor('A')
motorD = Motor('C')

# Sempre liberar pot�ncia m�xima
motorE.plimit(1.0)
motorD.plimit(1.0)


def mover(velE, velD):
    """
    velE e velD: -100 a 100

    Esquerda:
        +100 = frente
        -100 = r�

    Direita:
        +100 = frente
        -100 = r�
    """

    # Normaliza
    velE = max(-100, min(100, velE))
    velD = max(-100, min(100, velD))

    # Mapeia para faixa pwm (-1 a 1)
    pwmE = velE / 100.0

    # Inverte o motor direito
    pwmD = -velD / 100.0

    motorE.plimit(1.0)
    motorD.plimit(1.0)

    motorE.pwm(pwmE)
    motorD.pwm(pwmD)


def parar():
    motorE.pwm(0)
    motorD.pwm(0)







"""mover(50, 50)    # frente m�ximo
time.sleep(3.0)
mover(-50, -50)  # r� m�ximo
time.sleep(3.0)
mover(50, 50)      # frente 50%
time.sleep(3.0)
mover(100, -100)   # gira para direita
time.sleep(3.0)"""
mover(-100, 100)   # gira para esquerda
time.sleep(3.0)
parar()