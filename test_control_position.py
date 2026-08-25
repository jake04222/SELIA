import serial
import time
import math

# --- CONFIGURACIÓN FÍSICA (10 PULGADAS) ---
DIAMETRO_RUEDA_M = 0.254 
PPR = 16384  
CIRCUNFERENCIA = math.pi * DIAMETRO_RUEDA_M
PULSOS_POR_METRO = PPR / CIRCUNFERENCIA

# --- VARIABLES DE CONTROL ---
PORT = 'COM9' 
BAUDRATE = 115200
SLAVE_ID = 1

# AJUSTAR RAMPA Y RPM
RAMPA_MS = 500  # Tiempo en milisegundos para llegar a la velocidad máxima
VEL_RPM = 30     # Velocidad de crucero

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
    trama_completa = trama_base + calcular_crc(trama_base)
    ser.write(trama_completa)
    time.sleep(0.05)

def int_a_4bytes(n):
    """Convierte posición (puede ser negativa) a 32 bits (High 16, Low 16)"""
    # signed=True permite manejar el signo para la rueda invertida
    b = n.to_bytes(4, 'big', signed=True)
    return b[0:2], b[2:4]

def mover_casperbot_recto(metros):
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        
        # Calculo de pulsos
        # inviertir los signos dependiendo si avanza atras a adelante: (-pulsos, pulsos)
        pulsos_base = int(metros * PULSOS_POR_METRO)
        pulsos_izq = pulsos_base
        pulsos_der = -pulsos_base 
        
        print(f"--- Moviendo {metros}m | Rampa: {RAMPA_MS}ms ---")

        # 1. Modo Sincronizado y Modo Relativo
        enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0F, 0x00, 0x01]))
        enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0D, 0x00, 0x01]))
        
        # 2. Configurar RAMPA VARIABLE (Acel/Dec de ambos motores)
        # 2080h, 2081h, 2082h, 2083h
        r_bytes = RAMPA_MS.to_bytes(2, 'big')
        print(f"Configurando rampas a {RAMPA_MS} ms...")
        for reg in [0x80, 0x81, 0x82, 0x83]:
            enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, reg]) + r_bytes)

        # 3. Velocidad de Perfil (208Eh y 208Fh)
        v_bytes = VEL_RPM.to_bytes(2, 'big')
        enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x8E]) + v_bytes)
        enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x8F]) + v_bytes)

        # 4. Enable
        enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x08]))
        time.sleep(0.1)

        # 5. CARGAR POSICIONES INDEPENDIENTES (208Ah - 208Dh)
        h_izq, l_izq = int_a_4bytes(pulsos_izq)
        h_der, l_der = int_a_4bytes(pulsos_der)
        
        # Trama de 8 bytes: [Izq_H, Izq_L, Der_H, Der_L]
        trama_pos = bytearray([SLAVE_ID, 0x10, 0x20, 0x8A, 0x00, 0x04, 0x08])
        trama_pos += h_izq + l_izq + h_der + l_der
        enviar_trama(ser, trama_pos)

        # 6. ¡START!
        print("Enviando START (0x10)...")
        enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x10]))

        # Esperar llegada
        tiempo_viaje = (abs(metros) / (CIRCUNFERENCIA * (VEL_RPM/60))) + (RAMPA_MS/1000) + 1
        time.sleep(tiempo_viaje)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x07]))
            ser.close()
            print("Ciclo finalizado.")

if __name__ == "__main__":
    # Con este parametro comfiguramos la distancia que se quiere recorrer, ejm: 1 metro = 1.0
    mover_casperbot_recto(-0.1) # 10 centimetros
    #mover_casperbot_recto(-0.5125) # media revolucion
    #mover_casperbot_recto(0.2695) # 1/4 de rev
    #mover_casperbot_recto(0.95) # para testear
