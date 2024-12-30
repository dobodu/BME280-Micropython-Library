# free adaptation addition by Dobodu
#
# Based on Paul Cunnane 2016, Peter Dahlebrg 2016,
# Tony DiCola, David J Taylor and y Kevin Townsend.

LIBNAME = "BME280"
LIBVERSION = "1.0.0"


from ustruct import unpack_from

# BME280 default address.
BME280_DEFAULT_ADDRESS = (0x76, 0x77)

# Operating Modes
BME280_SAMPLING_1 = 1
BME280_SAMPLING_2 = 2
BME280_SAMPLING_4 = 3
BME280_SAMPLING_8 = 4
BME280_SAMPLING_16 = 5

#REGISTER MAP
BME280_REG_ID = 0xD0
BME280_REG_RESET = 0xE0
BME280_REG_CTRL_HUM = 0xF2
BME280_REG_STATUS = 0xF3
BME280_REG_CTRL_MES = 0xF4
BME280_REG_CTRL_MES_MODE_SLEEP = 0b00
BME280_REG_CTRL_MES_MODE_FORCE = 0b01
BME280_REG_CTRL_MES_MODE_NORMA = 0b11

BME280_REG_CONFIG = 0xF5
BME280_REG_PRESS = 0xF7 #20 bits
BME280_REG_TEMP = 0xFA #20 bits
BME280_REG_TEMP = 0xFD #16 bits

BME280_REG_CALIB00 = 0x88
BME280_REG_CALIB26 = 0xE1

SEA_LEVEL_PRESS = 1013.25

class BME280:

    def __init__(self, i2c, address=None, mode=BME280_SAMPLING_1, debug=False):
        
        self._debug = debug
        self._ready = False
        self._i2c = i2c
        self._mode = mode
        self._sea_level_press = SEA_LEVEL_PRESS
        
        # buffers and local variables
        self._buffer_1 = bytearray(1)
        self._buffer_8 = bytearray(8)
        self.t_fine = 0
        
        if address is None :
            devices = set(self._i2c.scan())
            mpus = devices.intersection(set(BME280_DEFAULT_ADDRESS))
            nb_of_mpus = len(mpus)
            if nb_of_mpus == 0:
                raise ValueError("No BME280 detected")
            elif nb_of_mpus == 1:
                self._bme_add = mpus.pop()
                self._dbg("BME280 : DEVICE FOUND AT ADDRESS... ",hex(self._bme_add))
                self._ready = True
            else:
                raise ValueError("Two BME280 detected: must specify a device address")
        else :
            self._bme_add = address    
        
        #Read calibration data
        buffer = self._i2c.readfrom_mem(self._bme_add, BME280_REG_CALIB00, 26)
        self.dig_T1, self.dig_T2, self.dig_T3, self.dig_P1, \
            self.dig_P2, self.dig_P3, self.dig_P4, self.dig_P5, \
            self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9, \
            _, self.dig_H1 = unpack_from("<HhhHhhhhhhhhBb", buffer)
        
        buffer = self._i2c.readfrom_mem(self._bme_add, BME280_REG_CALIB26, 7)
        self.dig_H2, self.dig_H3, e4, e5, e6, self.dig_H6= unpack_from("<hBBBBb", buffer)
        self.dig_H4 = (e4 << 4) | (e5 & 0xF)
        self.dig_H5 = (e6 << 4) | (e5 >> 4)
        
        #Setup oversampling
        #Humidity first
        self._buffer_1[0] = mode
        self._i2c.writeto_mem(self._bme_add, BME280_REG_CTRL_HUM, self._buffer_1)
        #Temp and Pressure Next
        ovs_temp = mode << 5
        ovs_pres = mode << 2
        reg_ctrl_mode = BME280_REG_CTRL_MES_MODE_NORMA
        self._buffer_1[0] = ovs_temp | ovs_pres | reg_ctrl_mode
        self._i2c.writeto_mem(self._bme_add, BME280_REG_CTRL_MES, self._buffer_1)


    def read_raw(self):

        #burst readout from 0xF7 to 0xFE
        self._i2c.readfrom_mem_into(self._bme_add,BME280_REG_PRESS, self._buffer_8)
        raw_press = ((self._buffer_8[0] << 16) | (self._buffer_8[1] << 8) | self._buffer_8[2]) >> 4
        raw_temp = ((self._buffer_8[3] << 16) | (self._buffer_8[4] << 8) | self._buffer_8[5]) >> 4
        raw_hum = (self._buffer_8[6] << 8) | self._buffer_8[7]

        return raw_temp, raw_press, raw_hum

    def read_compensated(self):

        raw_temp, raw_press, raw_hum = self.read_raw()
        # temperature
        var1 = ((raw_temp >> 3) - (self.dig_T1 << 1)) * (self.dig_T2 >> 11)
        var2 = (((((raw_temp >> 4) - self.dig_T1) *
                  ((raw_temp >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8

        # pressure
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = (((var1 * var1 * self.dig_P3) >> 8) +
                ((var1 * self.dig_P2) << 12))
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        if var1 == 0:
            pressure = 0
        else:
            p = 1048576 - raw_press
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P8 * p) >> 19
            pressure = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)

        # humidity
        h = self.t_fine - 76800
        h = (((((raw_hum << 14) - (self.dig_H4 << 20) -
                (self.dig_H5 * h)) + 16384)
              >> 15) * (((((((h * self.dig_H6) >> 10) *
                            (((h * self.dig_H3) >> 11) + 32768)) >> 10) +
                          2097152) * self.dig_H2 + 8192) >> 14))
        h = h - (((((h >> 15) * (h >> 15)) >> 7) * self.dig_H1) >> 4)
        h = 0 if h < 0 else h
        h = 419430400 if h > 419430400 else h
        humidity = h >> 12

        return temp, pressure, humidity           

    def set_press_offset(self):
        t, p, h = self.read_compensated()
        self._sea_level_press = int((p >> 8) / 100)

    @property
    def values(self):
        
        t, p, h = self.read_compensated()

        t = round(t/100,1)      # temperature in interally x100, keep 1 decimal
        p = int((p >> 8) / 100) # shif 8 bits fractionnal part and transform into hPa
        h = h >> 10             # shift 10 its fractionnal part 

        alt = int(44250 * (1 - ((p / self._sea_level_press) ** 0.189036)))

        return t, p, h, alt
    
    @property
    def ready(self):
        return self._ready
    
    def _dbg(self, *args, **kwargs):
        if self._debug:
            print("DBG:\tLIBNAME:\t", *args, **kwargs)