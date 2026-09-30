import serial
import time
import math

# --- CONFIGURACIÓN FÍSICA ---
DIAMETRO_RUEDA_M = 0.254 
PPR = 16384  
CIRCUNFERENCIA = math.pi * DIAMETRO_RUEDA_M
PULSOS_POR_METRO = PPR / CIRCUNFERENCIA

# --- VARIABLES DE CONTROL ---
PORT = '/dev/ttyUSB0' 
BAUDRATE = 115200
SLAVE_ID = 1

# AJUSTAR RAMPA Y RPM
RAMPA_MS = 500  
VEL_RPM = 30     

# --- DIRECCIONES MODBUS (Ajustar según manual de tu hardware) ---
# Ejemplos de direcciones relativas comunes:
REG_SALIDAS = 0x0000       # Registro o dirección para escribir salidas (Relés 9 y 10)
REG_ENTRADAS = 0x0000      # Registro para leer entradas digitales (Botonera, E-Stop)

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
    # Opcional: Leer respuesta si se requiere validación

def int_a_4bytes(n):
    b = n.to_bytes(4, 'big', signed=True)
    return b[0:2], b[2:4]

def limpiar_errores(ser):
    print("\n[!] Enviando comando CLEAR FAULT (0x200E -> 0x0006)...")
    enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x06]))
    print("[✓] Comando enviado.")

def leer_estado_pin(ser, pin_num):
    """
    Lee el estado de una entrada digital mediante Modbus (Función 0x02).
    Ajusta la dirección base del pin según tu tarjeta de E/S.
    """
    try:
        # Función 0x02: Read Discrete Inputs
        # [SLAVE_ID, 0x02, Addr_H, Addr_L, Cant_H, Cant_L]
        dir_pin = pin_num  # Dependiendo del PLC/Tarjeta, el pin 22 puede ser la dirección 22 o análoga
        trama = bytearray([SLAVE_ID, 0x02, (dir_pin >> 8) & 0xFF, dir_pin & 0xFF, 0x00, 0x01])
        trama_completa = trama + calcular_crc(trama)
        
        ser.reset_input_buffer()
        ser.write(trama_completa)
        time.sleep(0.02)
        respuesta = ser.read(6) # Respuesta típica: [Slave, 0x02, BytesCount, Data, CRC_L, CRC_H]
        
        if len(respuesta) >= 4:
            estado = (respuesta[3] & 0x01) # Asumiendo que el bit 0 trae el estado
            return bool(estado)
    except Exception as e:
        print(f"Error leyendo pin {pin_num}: {e}")
    return False

def controlar_salida(ser, salida_num, estado):
    """
    Controla una salida a relé (Función 0x05 para bobina única).
    """
    try:
        val_estado = 0xFF00 if estado else 0x0000
        # Función 0x05: Write Single Coil
        trama = bytearray([SLAVE_ID, 0x05, (salida_num >> 8) & 0xFF, salida_num & 0xFF, 
                           (val_estado >> 8) & 0xFF, val_estado & 0xFF])
        enviar_trama(ser, trama)
    except Exception as e:
        print(f"Error escribiendo salida {salida_num}: {e}")

def mover_casperbot_recto(ser, metros):
    try:
        pulsos_base = int(metros * PULSOS_POR_METRO)
        pulsos_izq = pulsos_base
        pulsos_der = -pulsos_base 
        
        print(f"--- Moviendo {metros}m | Rampa: {RAMPA_MS}ms ---")

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

        # 6. ¡START!
        enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x10]))

        # Esperar llegada controlando emergencias en tiempo real
        tiempo_viaje = (abs(metros) / (CIRCUNFERENCIA * (VEL_RPM/60))) + (RAMPA_MS/1000) + 1
        tiempo_inicio = time.time()
        
        while time.time() - tiempo_inicio < tiempo_viaje:
            # Monitorear Parada de Emergencia (Entrada 22) durante el movimiento
            if leer_estado_pin(ser, 22):
                print("[!] ¡EMERGENCIA DETECTADA DURANTE EL MOVIMIENTO!")
                controlar_salida(ser, 9, False)   # Apagar actuador salida 9
                controlar_salida(ser, 10, True)   # Activar relé salida 10
                raise Exception("Parada de Emergencia Activada")
            time.sleep(0.05)

    except Exception as e:
        print(f"Aviso de movimiento: {e}")
        raise e
    finally:
        # Detener motores al finalizar o ante error
        enviar_trama(ser, bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x07]))

def ciclo_principal():
    print("Iniciando sistema de control Casperbot...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(1)
        
        # Limpiar errores al arrancar
        limpiar_errores(ser)
        
        print("\n[✓] Sistema listo. Esperando Pulsador Verde...")

        while True:
            # Simular lectura de Botón Verde de arranque (Supongamos entrada digital ej: Pin 20)
            # Cambia '20' por la entrada física real de tu pulsador verde
            pulsador_verde = leer_estado_pin(ser, 20) 
            
            if pulsador_verde:
                print("\n[+] ¡Pulsador verde presionado! Arrancando ciclo...")
                
                while True:
                    # 1. Verificar Parada de Emergencia (Entrada 22) antes/durante el ciclo
                    if leer_estado_pin(ser, 22):
                        print("\n[!] ¡¡PARADA DE EMERGENCIA ACTIVADA (Entrada 22)!!")
                        controlar_salida(ser, 9, False)  # Apagar actuador (Salida 9)
                        controlar_salida(ser, 10, True)  # Activar salida de emergencia (Salida 10)
                        break # Rompe el ciclo de trabajo y va a espera segura

                    # 2. Verificar Botón de Parada (Botón 27)
                    if leer_estado_pin(ser, 27):
                        print("\n[!] Botón de parada 27 presionado. Deteniendo máquina...")
                        controlar_salida(ser, 9, False)
                        break

                    # 3. Realizar avance de 10cm con las llantas
                    print("--> Avanzando 10 cm...")
                    mover_casperbot_recto(ser, -0.1)

                    # Verificar E-Stop antes de activar brocas
                    if leer_estado_pin(ser, 22):
                        controlar_salida(ser, 9, False)
                        controlar_salida(ser, 10, True)
                        break

                    # 4. Activar el actuador de las brocas (Salida 9)
                    print("--> Activando actuador de brocas (Salida 9)...")
                    controlar_salida(ser, 9, True)
                    
                    # Simular tiempo de trabajo de las brocas (ajustar según proceso real)
                    tiempo_brocas = 3.0 
                    t_broca_inicio = time.time()
                    while time.time() - t_broca_inicio < tiempo_brocas:
                        if leer_estado_pin(ser, 22):
                            print("\n[!] ¡Emergencia durante proceso de brocas!")
                            controlar_salida(ser, 9, False)
                            controlar_salida(ser, 10, True)
                            raise Exception("Emergencia E-Stop")
                        time.sleep(0.05)

                    # Apagar actuador al finalizar proceso de brocas de este ciclo
                    controlar_salida(ser, 9, False)
                    print("--> Ciclo parcial completado. Reiniciando proceso...")

            time.sleep(0.1) # Pausa leve del hilo principal para no saturar el puerto serie

    except KeyboardInterrupt:
        print("\nPrograma detenido manualmente por el usuario.")
    except Exception as e:
        print(f"\n[X] Error crítico en la ejecución: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Puerto serie cerrado.")

if __name__ == "__main__":
    ciclo_principal()