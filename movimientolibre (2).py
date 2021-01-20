import sys
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
import smbus
import time
import curses
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
stdscr.refresh()

key = ''
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)
# This is the address we setup in the Arduino Program
address = 0x04
#Salidas para control de direccion
#motor1
GPIO.setup(8,GPIO.OUT)
GPIO.setup(10,GPIO.OUT)
#motor2
GPIO.setup(12,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)
#motor3
GPIO.setup(13,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)
#motor4
GPIO.setup(19,GPIO.OUT)
GPIO.setup(23,GPIO.OUT)
#Entrada para medir RE
#Encoder1
GPIO.setup(22,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
#Encoder2
GPIO.setup(24,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
#Encoder3
GPIO.setup(26,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
#Encoder4
GPIO.setup(21,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
#contador de rising edges
#funcion para escribir por i2c
def writeNumber(value):
    bus.write_byte(address, value)
    return -1
#Funcion para dar direccion al motor
#Parametros Numero direccion
def motorControl(arg1):
    #Programaci  n flecha enfrente
    if arg1==0:
        #M1
        GPIO.output(8,True)
        GPIO.output(10,False)
        #M2
        GPIO.output(12,True)
        GPIO.output(16,False)
        #M3
        GPIO.output(13,False)
        GPIO.output(15,True)
        #M4
        GPIO.output(19,False)
        GPIO.output(23,True)
    #Flecha atrs
    elif arg1==1:
         #M1
        GPIO.output(8,False)
        GPIO.output(10,True)
        #M2
        GPIO.output(12,False)
        GPIO.output(16,True)
        #M3
        GPIO.output(13,True)
        GPIO.output(15,False)
        #M4
        GPIO.output(19,True)
        GPIO.output(23,False)
    #Flecha Izquierda
    elif arg1==2:
         #M1
        GPIO.output(8,True)
        GPIO.output(10,False)
        #M2
        GPIO.output(12,False)
        GPIO.output(16,True)
        #M3
        GPIO.output(13,True)
        GPIO.output(15,False)
        #M4
        GPIO.output(19,False)
        GPIO.output(23,True)
    #Flecha derecha
    elif arg1==3:
         #M1
        GPIO.output(8,False)
        GPIO.output(10,True)
        #M2
        GPIO.output(12,True)
        GPIO.output(16,False)
        #M3
        GPIO.output(13,False)
        GPIO.output(15,True)
        #M4
        GPIO.output(19,True)
        GPIO.output(23,False)
    #up
    elif arg1==4:
         #M1
        GPIO.output(8,True)
        GPIO.output(10,False)
        #M2
        GPIO.output(12,True)
        GPIO.output(16,False)
        #M3
        GPIO.output(13,True)
        GPIO.output(15,False)
        #M4
        GPIO.output(19,True)
        GPIO.output(23,False)
    #down
    elif arg1==5:
         #M1
        GPIO.output(8,False)
        GPIO.output(10,True)
        #M2
        GPIO.output(12,False)
        GPIO.output(16,True)
        #M3
        GPIO.output(13,False)
        GPIO.output(15,True)
        #M4
        GPIO.output(19,False)
        GPIO.output(23,True)
    return;
#Funcion de controlador PID
def pid_controller(y, yc, h=1, Ti=0.5, Td=0.1, Kp=1000, u0=0, e0=0):
    # Step variable
    k = 0
    # Initialization
    ui_prev = u0
    e_prev = e0
    # Error between the desired and actual output
    e = yc - y
    # Integration Input
    ui = ui_prev + 1/Ti * h*e
    # Derivation Input
    ud = 1/Td * (e - e_prev)/h
    # Adjust previous values
    e_prev = e
    ui_prev = ui
    # Calculate input for the system
    u = Kp * (e + ui + ud)
    k += 1

    return u
#Funcion que se detiene de acuardo a los pulsos detectados
#Parametros numero de motor y pasos
def encoder(arg1):
#senal para detener sistema
    stop=0
    stop0=0
    stop1=0
    stop2=0
    stop3=0
#contadores de rising edges
    c0=0
    c1=0
    c2=0
    c3=0
#Valores de x presente
    xp0=GPIO.input(22)
    xp1=GPIO.input(24)
    xp2=GPIO.input(26)
    xp3=GPIO.input(21)
    while stop==0:
        #valores de x nuevo
        xf0=GPIO.input(22)
        xf1=GPIO.input(24)
        xf2=GPIO.input(26)
        xf3=GPIO.input(21)
        #comparaciones de valores pasado y actual
        x0=xp0^xf0
        xp0=xf0
        x1=xp1^xf1
        xp1=xf1
        x2=xp2^xf2
        xp2=xf2
        x3=xp3^xf3
        xp3=xf3
        #Conteo de rising edges
        if x0:
            c0+=1
        control0=pid_controller(c0,arg1)
        if x1:
            c1+=1
        control1=pid_controller(c1,arg1)
        if x2:
            c2+=1
        control2=pid_controller(c2,arg1)
        if x3:
            c3+=1
        control3=pid_controller(c3,arg1)
        if control0==0:
            #M1
            stop0=1
            GPIO.output(8,False)
            GPIO.output(10,False)
        if control1==0:
            #M2
            stop1=1
            GPIO.output(12,False)
            GPIO.output(16,False)
        if control2==0:
            #M3
            stop2=1
            GPIO.output(13,False)
            GPIO.output(15,False)
        if control3==0:
            #M4
            stop3=1
            GPIO.output(19,False)
            GPIO.output(23,False)
        if stop0&stop1&stop2&stop3:
            stop=1
    #termina el PWM y limpia los puertos
    writeNumber(0)
    return;
#funcion que integra todas las funciones de movimiento
#arg1 = num motor
def motor(arg1):
    #Escribe la velocidad a la tarjeta de rafa por i2c
    writeNumber(100)
    #Movimento flecha arriba 
    motorControl(arg1)
    #Detiene el motor de acuerdo al numero de pasos enviado
    encoder(222)
    return;
#Ejecucion de funciones
try:
    #Introduccion de parametros desde la consola
    a=sys.argv
    #Seleccion de moviemintos con teclas
    while key != ord('q'):
        key=stdscr.getch()
        #Tecla up
        if key == 259:
            motor(0)
        #Tecla down
        elif key== 258:
            motor(1)
        #Tecla left
        elif key== 260:
            motor(2)
        #Tecla right
        elif key== 261:
            motor(3)
        #up
        elif key== ord('u'):
            motor(4)
        #down
        elif key== ord('d'):
            motor(5)
    GPIO.cleanup()
    curses.endwin()
#Rutina ctrl+c
except KeyboardInterrupt:
    GPIO.cleanup()
    curses.endwin()
