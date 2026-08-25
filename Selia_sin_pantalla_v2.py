import os
import time
import math
import serial
import threading
from gpiozero import Button

# --- CONFIGURACIÓN FÍSICA Y MODBUS ---
DIAMETRO_RUEDA_M = 0.254 
PPR = 16384  
CIRCUNFERENCIA = math.pi * DIAMETRO_RUEDA_M
PULSOS_POR_METRO = PPR / CIRCUNFERENCIA

DISTANCIA_MOVIMIENTO = 0.1 

# Si usas un adaptador USB a RS485/Serie, usa '/dev/ttyUSB0'
PORT = '/dev/ttyUSB0' 
BAUDRATE = 115200
SLAVE_ID = 1

RAMPA_MS = 500  
VEL_RPM = 30     

# --- CONFIGURACIÓN DE PINES GPIO (BCM) ---
PIN_PULSADOR_INICIO = 17   # GPIO 17 (Pin físico 11)
PIN_PULSADOR_DETENER = 27  # GPIO 27 (Pin físico 13)

# Variables de estado global
ejecutando_secuencia = False
detener_solicitado = False
ser_global = None

def calcular_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')

def enviar_trama(ser, trama_base):
    if ser and ser.is_open:
        trama_completa = trama_base + calcular_crc(trama_base)
        ser.write(trama_completa)
        time.sleep(0.05)

def int_a_4bytes(n):
    """Convierte posición (puede ser negativa) a 32 bits (High 16, Low 16)"""
    b = n.to_bytes(4, 'big', signed=True)
    return b[0:2], b[2:4]

def detener_motores(ser):
    """Envía la trama de parada/disable (0x200E -> 0x0007)"""
    print("\n[!] Deteniendo motores...")
    enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x07]))

def limpiar_errores(ser):
    print("\n[!] Enviando comando CLEAR FAULT (0x200E -> 0x0006)...")
    enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x06]))
    print("[✓] Errores limpiados.")

def mover_robot_recto(ser, metros):
    global detener_solicitado
    if detener_solicitado:
        return False

    pulsos_base = int(metros * PULSOS_POR_METRO)
    pulsos_izq = pulsos_base
    pulsos_der = -pulsos_base 

    print(f"\n--- Moviendo {metros}m | Rampa: {RAMPA_MS}ms ---")

    # 1. Modo Sincronizado y Modo Relativo
    enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0F, 0x00, 0x01]))
    enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0D, 0x00, 0x01]))

    # 2. Configurar RAMPA VARIABLE
    r_bytes = RAMPA_MS.to_bytes(2, 'big')
    for reg in [0x80, 0x81, 0x82, 0x83]:
        enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, reg]) + r_bytes)

    # 3. Velocidad de Perfil
    v_bytes = VEL_RPM.to_bytes(2, 'big')
    enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x8E]) + v_bytes)
    enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x8F]) + v_bytes)

    # 4. Enable
    enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x08]))
    time.sleep(0.1)

    # 5. CARGAR POSICIONES
    h_izq, l_izq = int_a_4bytes(pulsos_izq)
    h_der, l_der = int_a_4bytes(pulsos_der)
    trama_pos = bytearray([SLAVE_ID, 0x10, 0x20, 0x8A, 0x00, 0x04, 0x08]) + h_izq + l_izq + h_der + l_der
    enviar_trama(ser, trama_pos)

    # 6. START
    enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x10]))

    # Esperar el tiempo de movimiento verificando interrupción de parada
    tiempo_viaje = (abs(metros) / (CIRCUNFERENCIA * (VEL_RPM / 60))) + (RAMPA_MS / 1000) + 0.5
    inicio = time.time()
    
    while time.time() - inicio < tiempo_viaje:
        if detener_solicitado:
            detener_motores(ser)
            return False
        time.sleep(0.05)

    return True

def rutina_perforacion():
    """Bucle continuo de movimiento y perforación"""
    global ejecutando_secuencia, detener_solicitado, ser_global
    
    # Requisito: Limpiar errores al iniciar el ciclo antes de avanzar
    limpiar_errores(ser_global)
    time.sleep(0.1)

    while ejecutando_secuencia and not detener_solicitado:
        # 1. Avanzar la distancia configurada
        exito = mover_robot_recto(ser_global, DISTANCIA_MOVIMIENTO)
        if not exito:
            break

        # 2. Parar 5 segundos e imprimir mensaje
        print("\n[+] Bajando broca y abriendo hueco...")
        inicio_espera = time.time()
        while time.time() - inicio_espera < 5.0:
            if detener_solicitado:
                break
            time.sleep(0.1)

        if detener_solicitado:
            break

    detener_motores(ser_global)
    ejecutando_secuencia = False
    print("\n--- Secuencia finalizada/detenida. Esperando pulsador de inicio ---")

# --- MANEJADORES DE EVENTOS DE BOTONES ---
def presionar_inicio():
    global ejecutando_secuencia, detener_solicitado, ser_global
    if not ejecutando_secuencia:
        print(f"\n[>] Pulsador 1 Presionado: Limpiando errores e iniciando secuencia ({DISTANCIA_MOVIMIENTO}m)...")
        detener_solicitado = False
        ejecutando_secuencia = True
        hilo_secuencia = threading.Thread(target=rutina_perforacion, daemon=True)
        hilo_secuencia.start()
    else:
        print("\n[!] La secuencia ya está en ejecución.")

def presionar_detener():
    global ejecutando_secuencia, detener_solicitado, ser_global
    print("\n[>] Pulsador 2 Presionado: Deteniendo secuencia...")
    detener_solicitado = True
    ejecutando_secuencia = False
    detener_motores(ser_global)

def main():
    global ser_global

    btn_inicio = Button(PIN_PULSADOR_INICIO, pull_up=True, bounce_time=0.1)
    btn_detener = Button(PIN_PULSADOR_DETENER, pull_up=True, bounce_time=0.1)

    btn_inicio.when_pressed = presionar_inicio
    btn_detener.when_pressed = presionar_detener

    try:
        ser_global = serial.Serial(PORT, BAUDRATE, timeout=1)
        print("=== CONTROL DE MOTOR ZLTECH (RASPBERRY PI 4) ===")
        print(f"Puerto Serie: {PORT}")
        print(f"Distancia configurada por ciclo: {DISTANCIA_MOVIMIENTO}m")
        print("Pulsador 1 (GPIO 17): Iniciar ciclo continuo")
        print("Pulsador 2 (GPIO 27): Detener motores/secuencia")
        print("Presiona Ctrl+C en la terminal para cerrar el programa.")
        print("=================================================")

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nSaliendo del programa...")
    except Exception as e:
        print(f"\nError general: {e}")
    finally:
        if ser_global and ser_global.is_open:
            detener_motores(ser_global)
            ser_global.close()
            print("Puerto serie cerrado.")

if __name__ == "__main__":
    main()