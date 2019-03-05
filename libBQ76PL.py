import RPi.GPIO as GPIO
import serial
import time
import math
from crccheck.crc import CrcArc
from crccheck.checksum import ChecksumXor16

class BQ76PL:
    """Biblioteca de comunicaciones de la placa de Evaluación BQ76PL455AQ1 (RPi)

    La creación de esta biblioteca tiene como objetivo facilitar la comunicación con
    el módulo de evaluación de Texas Instrument BQ76PL455AQ1 a traves del puerto serie
    de una placa Raspberry Pi.

    Esta biblioteca tiene integrada una serie de funciones de comunicaciones generales
    acompañadas de otras funciones simplificadas para las operaciones más comunes con el
    módulo.
    """
    port = "/dev/serial0"
    baudrate = 250000
    timeout = 3.0
    wakePin = 4

    def __init__(self, port = "/dev/serial0", baudrate = 250000, timeout = 3.0, wakePin = 4, idle = False):
        """Crear un objeto de la clase BQ76PL e inicia la comunicación
    
        Keyword Arguments:
            port {str} -- Puerto serie conectado (default: {"/dev/serial0"})
            baudrate {int} --  (default: {250000})
            timeout {float} --  (default: {3.0})
            wakePin {int} --  (default: {4})
            idle{boolean} -- Inicia o no las comunicaciones (default: {False})
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.wakePin = wakePin

        self.__wakePin = wakePin
        if not idle:
            self.startComm()

    def startComm(self):
        """Inicia las comunicaciones, solo necesario si se han cerrado anteriormente
        """
        self.__comm = serial.Serial(self.port, baudrate=self.baudrate, timeout=self.timeout)

    def wake(self):
        """Saca el dispostivo del estado Idle o Shutdown y lo prepara para las comunicaciones.
        Los comandos de comunicaciones lo llaman cada vez que estos lo son, por lo tanto no es necesario
        que el usuario use wake() antes de las comunicaciones.
        """
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__wakePin, GPIO.OUT)
        GPIO.output(self.__wakePin, GPIO.LOW)
        time.sleep(1)
        GPIO.output(self.__wakePin, GPIO.HIGH)

    def shutdown(self):
        """Pone todos los módulos de evaluación conectados al puerto serie
        en shutdown (mediante un comando tipo "Broadcast"). En este estado se desconectan las comunicaciones
        y el dispositivo para a modo de bajo consumo.
        """
        self.sendMessage('BroadNoRes', 1, 0x0C, 0x40)

    def stopComm(self):
        """Cierra el puerto, liberándolo
        """
        self.__comm.close()

    def sendMessage(self, method, Nbytes, rAddress, data, dAddress = None):
        """Comando genérico de comunicaciones para el envio de datos a la placa de evalución.
        
        Arguments:
            method {string} -- Consultar el datasheet para más información de los metodos de escritura.
            Nbytes {int} -- Número de bytes a enviar. Los envios de 7 bytes no estan permitidos, si se intenta hacer se enviaran
        8 bytes, rellenando el byte más significativo con 0. Esto podría afectar a un registro no deseado. Usar con cuidado.
            rAddress {int,hex,bin} -- Dirección del registro de destino. Se pueden utilizar enteros, valores hexadecimales (de la forma 0x123AF) o
        valores binarios (de la forma 0b01110)
            data {int,hex,bin} -- Datos a enviar, el tamaño debe coincidir con Nbytes.
        
        Keyword Arguments:
            dAddress {int,hex,bin} -- Parámetro opcional, obligatorio para las transiciones a un solo dispositivo o hacia un grupo, ignorado para los
        envios con el método 'Broadcast'. Contiene la dirección del dispositivo o del grupo. Puede estar expresado de la misma forma que
        rAddress. (default: {None})
        """
        self.wake()
        def binary(b):
            s = int(bin(b),2)
            return s
        methodsDic = {'SinDevWthRes' : 0b000,
                'SinDevNoRes' : 0b001,
                'GroupWthRes' : 0b010,
                'GroupNoRes' : 0b011,
                'BroadWthRes' : 0b110,
                'BroadNoRes' : 0b111}
        if Nbytes == 8:
            Nbytes = 7
        byteInit = (0b1 << 7) + (methodsDic[method] << 4) + (0b0 << 3) + binary(Nbytes)
        if method not in ['BroadWthRes', 'BroadNoRes']:
            sendingByte = ((binary(byteInit) << Nbytes*8 + 16) + (binary(dAddress) << Nbytes*8 + 8) + (binary(rAddress) << Nbytes*8) + binary(data)).to_bytes(3 + Nbytes,byteorder = 'big')
        else:
            sendingByte = ((binary(byteInit) << Nbytes*8 + 8) + (binary(rAddress) << Nbytes*8) + binary(data)).to_bytes(2 + Nbytes,byteorder = 'big')
        crc = (CrcArc.calc(sendingByte)).to_bytes(2,byteorder = 'little')
        sendingByte = sendingByte + crc
        self.__comm.write(sendingByte)
        return

    def reciveMessage(self):
        """Recibe un mensaje del dispositivo. Devuelve el dato recibido, 'CommErr' si no recibe nada o 'CrcErr' si hay un error en
        el proceso de comprobación del código de redundacia cíclica.
        
        Returns:
            hex,str -- El mensaje recibido o el error
        """
        self.wake()
        init = self.__comm.read(1)
        if init == b'':
            Message = 'CommErr'
        else:
            Message = self.__comm.read(int.from_bytes(init,'big')+3)
            preCRC = init + Message
            crc = CrcArc.calc(init + Message)
            if crc == 0:
                Message = bytearray(Message)
                Message = bytes(Message[:len(Message)-2])
            else:
                Message = 'CRCErr'
        return Message

    def getVoltage(self, cells, method = 'SinDevWthRes'):
        """Obtiene el voltaje actual de las celdas.
        
        Arguments:
            cells {array} -- Array formado por enteros de cada una de las celdas que queremos obtener su tensión. Solo están admitidos los números entre
        1 y 16. La función devuelve las tensiones en el mismo orden en que se colocan en el array.
        
        Keyword Arguments:
            method {str} -- Indica el metodo de la comunicación. (default: {'SinDevWthRes'})
        
        Returns:
            array -- Array con los voltajes. Ej,  getVoltage([2,5,3]) devuelve [V_2, V_5, V_3] siendo V_n el voltaje de la celda n.
        """
        sortCells = sorted(cells, reverse = True)
        data = 0
        for c in sortCells:
            if c > 16 or c < 1:
                print('Celda fuera de rango')
                return 'RangeErr'
            data = data + (1 << c + 15)
        self.sendMessage(method, 5, 0x02, data, 0x00)
        bytesV = bytearray(self.reciveMessage())
        intV = []
        for i in range(len(cells)):
            intV.append(int.from_bytes(bytesV[2*i:2*i+2],'big'))
            intV[i] = round(((intV[i])*(5/65535)),3)
        d = dict(zip(sortCells, intV))
        V=[]
        for i in range(len(cells)):
            V.append(d[cells[i]])
        return V

    def enableBalance(self, cells, time = 'Manual', method = 'SinDevNoRes'):
        """Activa el balanceo pasivo de las celdas seleccionadas, si ya había un balanceo activo
        el último sobreescribe a todos los anteriores y reinicia el timer.
            
        Arguments:
            cells {array} -- Celdas a balancear, igual que en getVoltage pueden ir en el orden que sea.
        
        Keyword Arguments:
            time {str} -- Activa el timer para controlar el tiempo máximo de balanceo, en modo manual solo se detendra con otra llamada de enableBalance o mediante stopBalance.
        Las opciones de tiempo son 'Manual', '1 s', '1 min', '2 min', '5 min', '10 min', '15 min', '20 min', '30 min' y '60 min'. (default: {'Manual'})
            method {str} -- (default: {'SinDevNoRes'})
        """
        
        timeDic= {'Manual' : 0b0000,
                '1 s' : 0b0001,
                '1 min' : 0b0010,
                '2 min' : 0b0011,
                '5 min' : 0b0100,
                '10 min' : 0b0101,
                '15 min' : 0b0110,
                '20 min' : 0b0111,
                '30 min' : 0b1000,            
                '60 min' : 0b1001}
        sortCells = sorted(cells, reverse = True)
        data = 0
        for c in sortCells:
            if c > 16 or c < 1:
                print('Celda fuera de rango')
                return 'RangeErr'
            data = data + (1 << c-1)
        data = (timeDic[time] << 20) + (1 << 19) + data
        self.sendMessage(method, 3, 0x13, data, 0x00)
        return

    def readBalance(self, method = 'SinDevWthRes'):
        """Devuelve un array con todas las celdas que se están balanceando.
        
        Keyword Arguments:
            method {str} -- (default: {'SinDevWthRes'})
        
        Returns:
            array -- Contiene las celdas que se estan balanceando
        """
        self.sendMessage(method, 1, 0x14, 0x01, 0x00)
        m = int.from_bytes(self.reciveMessage(),'big')
        cells = []
        i = 1
        while i <= m:
            if m & i:
                cells.append(i)
            i <<= 1
        cells = list(map(lambda x: int(math.log(x,2) + 1), cells))
        return cells

    def stopBalance(self, method = 'SinDevNoRes'):
        """Detiene el balanceo de todas las celdas.
        
        Keyword Arguments:
            method {str} -- (default: {'SinDevNoRes'})
        """
        self.sendMessage(method, 2, 0x14, 0x00, 0x00)
        return                

if __file__ == "__main__":
    EVM = BQ76PL()

    V = EVM.getVoltage([6,5,4,3,2,1])
    print(V)

    EVM.stopComm()
