import minimalmodbus
import serial
import time

PORTNAME = '/dev/ttyUSB0'
BAUDRATE = 19200
BYTESIZE = 8
PARITY = 'N'
STOPBITS = 1
TIMEOUT = 1
SLAVEADDRESS = 9

REGISTER_START = 180

global ForceSensorValue
ForceSensorValue = [0,0,0,0,0,0]

def setup_sensor():
    ft300 = minimalmodbus.Instrument(PORTNAME, SLAVEADDRESS)
    ft300.serial.baudrate = BAUDRATE
    ft300.serial.bytesize = BYTESIZE
    ft300.serial.parity = PARITY
    ft300.serial.stopbits = STOPBITS
    ft300.serial.timeout = TIMEOUT
    ft300.mode = minimalmodbus.MODE_RTU
    return ft300

def force_converter(raw_value):
    if raw_value > 32767:
        raw_value -= 65536
    force = raw_value / 100.0
    return force

def torque_converter(raw_value):
    if raw_value > 32767:
        raw_value -= 65536
    torque = raw_value / 1000.0
    return torque

def mainloop():
    print("Initializing FT300-S sensor...")
    try:
        print("Attempting to stop possible streaming mode...")
        ser = serial.Serial(
            port=PORTNAME,
            baudrate=BAUDRATE,
            bytesize=BYTESIZE,
            parity=PARITY,
            stopbits=STOPBITS,
            timeout=TIMEOUT
        )
        packet = bytearray([0xFF] * 50)
        ser.write(packet)
        ser.close()
        print("Stop streaming command sent.")
    except Exception as e:
        print(f"Exception when stopping stream mode (can be ignored): {e}")

    try:
        ft300 = setup_sensor()
        print(f"Sensor connected successfully! (Port: {PORTNAME})")

        initial_regs = ft300.read_registers(REGISTER_START, 6)
        fx_zero = force_converter(initial_regs[0])
        fy_zero = force_converter(initial_regs[1])
        fz_zero = force_converter(initial_regs[2])
        tx_zero = torque_converter(initial_regs[3])
        ty_zero = torque_converter(initial_regs[4])
        tz_zero = torque_converter(initial_regs[5])

        print("Sensor initialization complete. Starting data reading...\n")
        print("=" * 70)
        print("*** Press Ctrl+C to stop ***")
        print("=" * 70)

        global ForceSensorValue
        while True:
            registers = ft300.read_registers(REGISTER_START, 6)

            fx = force_converter(registers[0]) - fx_zero
            fy = force_converter(registers[1]) - fy_zero
            fz = force_converter(registers[2]) - fz_zero
            tx = torque_converter(registers[3]) - tx_zero
            ty = torque_converter(registers[4]) - ty_zero
            tz = torque_converter(registers[5]) - tz_zero

            #print(f"fx={fx:8.2f} N  fy={fy:8.2f} N  fz={fz:8.2f} N  "f"tx={tx:7.4f} Nm  ty={ty:7.4f} Nm  tz={tz:7.4f} Nm")
            ForceSensorValue = [fx,fy,fz,tx,ty,tz]
            #print("Sensor_Robotiq ForceSensorValue=" + str(ForceSensorValue[0])+ "," +str(ForceSensorValue[1])+ "," +str(ForceSensorValue[2]))

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except minimalmodbus.NoResponseError:
        print(f"Error: No response. Check: 1) Device powered? 2) Wiring correct? 3) Port name '{PORTNAME}' correct?")
    except Exception as e:
        print(f"Unknown error: {e}")
    finally:
        try:
            ft300.serial.close()
            print("Serial port closed.")
        except:
            pass

# if __name__ == "__main__":
#     main()
