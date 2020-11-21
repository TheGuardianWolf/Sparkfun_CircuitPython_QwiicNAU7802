import busio
import time
import board

NAU7802_DeviceAddress = 0x2A
NS_PER_MS = 1000000

# class Scale_Registers(IntEnum):
NAU7802_PU_CTRL = 0
NAU7802_CTRL1 = 1
NAU7802_CTRL2 = 2
NAU7802_OCAL1_B2 = 3
NAU7802_OCAL1_B1 = 4
NAU7802_OCAL1_B0 = 5
NAU7802_GCAL1_B3 = 6
NAU7802_GCAL1_B2 = 7
NAU7802_GCAL1_B1 = 8
NAU7802_GCAL1_B0 = 9
NAU7802_OCAL2_B2 = 10
NAU7802_OCAL2_B1 = 11
NAU7802_OCAL2_B0 = 12
NAU7802_GCAL2_B3 = 13
NAU7802_GCAL2_B2 = 14
NAU7802_GCAL2_B1 = 15
NAU7802_GCAL2_B0 = 16
NAU7802_I2C_CONTROL = 17
NAU7802_ADCO_B2 = 18
NAU7802_ADCO_B1 = 19
NAU7802_ADCO_B0 = 20
NAU7802_ADC = 21
NAU7802_OTP_B1 = 22
NAU7802_OTP_B0 = 23
NAU7802_PGA = 27
NAU7802_PGA_PWR = 28
NAU7802_DEVICE_REV = 31


# class PU_CTRL_Bits(IntEnum):
NAU7802_PU_CTRL_RR = 0
NAU7802_PU_CTRL_PUD = 1
NAU7802_PU_CTRL_PUA = 2
NAU7802_PU_CTRL_PUR = 3
NAU7802_PU_CTRL_CS = 4
NAU7802_PU_CTRL_CR = 5
NAU7802_PU_CTRL_OSCS = 6
NAU7802_PU_CTRL_AVDDS = 7


# class CTRL1_Bits(IntEnum):
NAU7802_CTRL1_GAIN = 2
NAU7802_CTRL1_VLDO = 5
NAU7802_CTRL1_DRDY_SEL = 6
NAU7802_CTRL1_CRP = 7


# class CTRL2_Bits(IntEnum):
NAU7802_CTRL2_CALMOD = 0
NAU7802_CTRL2_CALS = 2
NAU7802_CTRL2_CAL_ERROR = 3
NAU7802_CTRL2_CRS = 4
NAU7802_CTRL2_CHS = 7


# class PGA_Bits(IntEnum):
NAU7802_PGA_CHP_DIS = 0
NAU7802_PGA_INV = 3
NAU7802_PGA_BYPASS_EN = 4
NAU7802_PGA_OUT_EN = 5
NAU7802_PGA_LDOMODE = 6
NAU7802_PGA_RD_OTP_SEL = 7


# class PGA_PWR_Bits(IntEnum):
NAU7802_PGA_PWR_PGA_CURR = 0
NAU7802_PGA_PWR_ADC_CURR = 2
NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4
NAU7802_PGA_PWR_PGA_CAP_EN = 7


# class NAU7802_LDO_Values(IntFlag):
NAU7802_LDO_2V4 = 0b111
NAU7802_LDO_2V7 = 0b110
NAU7802_LDO_3V0 = 0b101
NAU7802_LDO_3V3 = 0b100
NAU7802_LDO_3V6 = 0b011
NAU7802_LDO_3V9 = 0b010
NAU7802_LDO_4V2 = 0b001
NAU7802_LDO_4V5 = 0b000


# class NAU7802_Gain_Values(IntFlag):
NAU7802_GAIN_128 = 0b111
NAU7802_GAIN_64 = 0b110
NAU7802_GAIN_32 = 0b101
NAU7802_GAIN_16 = 0b100
NAU7802_GAIN_8 = 0b011
NAU7802_GAIN_4 = 0b010
NAU7802_GAIN_2 = 0b001
NAU7802_GAIN_1 = 0b000


# class NAU7802_SPS_Values(IntFlag):
NAU7802_SPS_320 = 0b111
NAU7802_SPS_80 = 0b011
NAU7802_SPS_40 = 0b010
NAU7802_SPS_20 = 0b001
NAU7802_SPS_10 = 0b000


# class NAU7802_Channels(IntEnum):
NAU7802_CHANNEL_1 = 0
NAU7802_CHANNEL_2 = 1


# class NAU7802_Cal_Status(IntEnum):
NAU7802_CAL_SUCCESS = 0
NAU7802_CAL_IN_PROGRESS = 1
NAU7802_CAL_FAILURE = 2


class NAU7802(object):
    """Library written for the NAU7802 24-bit wheatstone bridge and load cell amplifier.

    The NAU7802 is an I2C device that converts analog signals to a 24-bit
    digital signal. This makes it possible to create your own digital scale
    either by hacking an off-the-shelf bathroom scale or by creating your
    own scale using a load cell.

    Ported from the Cpp version by Jerry Fan
    Cpp version By Nathan Seidle @ SparkFun Electronics, March 3nd, 2019
    https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library
    """

    def __init__(self):
        """Initialises the object with required devices

        Args:
            i2c (busio.I2C): [description]
        """
        self.zeroOffset = 0
        self.calibrationFactor = 1.0

    def begin(self, i2c: busio.I2C = None, reset: bool = True):
        """Check communication and initialize sensor

        Args:
            i2c (busio.I2C, optional): [description]. Defaults to None.
            reset (bool, optional): [description]. Defaults to True.

        Returns:
            [type]: [description]
        """
        if i2c is None:
            self.i2c = board.I2C()
            if not self.i2c.try_lock():
                return False
        else:
            self.i2c = i2c

        if not self.isConnected():
            if not self.isConnected():
                return False

        result = 1

        if reset:
            result &= self.reset()
            result &= self.powerUp()
            result &= self.setLDO(NAU7802_LDO_3V3)
            result &= self.setGain(NAU7802_GAIN_128)
            result &= self.setSampleRate(NAU7802_SPS_80)
            result &= self.setRegister(NAU7802_ADC, 0x30)
            result &= self.setBit(
                NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR)
            result &= self.calibrateAFE()

        return bool(result)

    def isConnected(self):
        """Returns true if device acks at the I2C address
        """
        try:
            self.i2c.writeto(NAU7802_DeviceAddress, bytearray())
            return True
        except OSError:
            return False

    def available(self):
        """Returns true if Cycle Ready bit is set (conversion is complete)
        """
        return self.getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL)

    def getReading(self):
        """Returns 24-bit reading. Assumes CR Cycle Ready bit (ADC conversion complete) has been checked by .available()
        """
        try:
            self.i2c.writeto(NAU7802_DeviceAddress, bytearray(
                [NAU7802_ADCO_B2]))

            read_buffer = bytearray(3)
            self.i2c.readfrom_into(NAU7802_DeviceAddress, read_buffer)

            sign = (read_buffer[0] >> 7) & 0x01
            value = (read_buffer[0] << 16) | (
                read_buffer[1] << 8) | (read_buffer[2])
            if sign:
                # Two's compliment (24-bits)
                value = -(((~value) & 0xFFFFFF) + 1)
            return value
        except OSError:
            return 0

    def getAverage(self, samplesToTake: int, timeout_ms=5000):
        """Return the average of a given number of readings

        Args:
            samplesToTake (int): [description]
        """
        total = 0
        samplesAquired = 0

        startTime = time.monotonic_ns() // NS_PER_MS
        while True:
            if self.available():
                total += self.getReading()
                samplesAquired += 1
                if samplesAquired >= samplesToTake:
                    break
            if (time.monotonic_ns() // NS_PER_MS - startTime) > timeout_ms:
                return 0
            time.sleep(0.001)
        total /= samplesAquired

        return int(total)

    def calculateZeroOffset(self, averageAmount: int = 8):
        """Also called taring. Call this with nothing on the scale

        Args:
            averageAmount (int, optional): [description]. Defaults to 8.
        """
        self.setZeroOffset(self.getAverage(averageAmount))

    def setZeroOffset(self, newZeroOffset: int):
        """Sets the internal variable. Useful for users who are loading values from NVM.

        Args:
            newZeroOffset (int): [description]
        """
        self.zeroOffset = newZeroOffset

    def getZeroOffset(self):
        """Ask library for this value. Useful for storing value into NVM.
        """
        return self.zeroOffset

    def calculateCalibrationFactor(self, weightOnScale: float, averageAmount: int = 8):
        """Call this with the value of the thing on the scale. Sets the calibration factor based on the weight on scale and zero offset.

        Args:
            weightOnScale (float): [description]
            averageAmount (int, optional): [description]. Defaults to 8.
        """
        onScale = self.getAverage(averageAmount)
        newCalFactor = (onScale - self.zeroOffset) / weightOnScale
        self.setCalibrationFactor(newCalFactor)

    def setCalibrationFactor(self, calFactor: float):
        """Pass a known calibration factor into library. Helpful if users is loading settings from NVM.

        Args:
            calFactor (float): [description]
        """
        self.calibrationFactor = calFactor

    def getCalibrationFactor(self):
        """Ask library for this value. Useful for storing value into NVM.
        """
        return self.calibrationFactor

    def getWeight(self, allowNegativeWeights: bool = False, samplesToTake: int = 8):
        """Once you've set zero offset and cal factor, you can ask the library to do the calculations for you.

        Args:
            allowNegativeWeights ([type], optional): [description]. Defaults to False:bool.
            samplesToTake (int, optional): [description]. Defaults to 8.
        """
        onScale = self.getAverage(samplesToTake)

        if not allowNegativeWeights:
            if onScale < self.zeroOffset:
                onScale = self.zeroOffset

        weight = (onScale - self.zeroOffset) / self.calibrationFactor
        return weight

    def setGain(self, gainValue: int):
        """Set the gain. x1, 2, 4, 8, 16, 32, 64, 128 are available

        Args:
            gainValue (int): [description]
        """
        if gainValue > 0b111:
            gainValue = 0b111

        value = self.getRegister(NAU7802_CTRL1)
        value &= 0b11111000
        value |= gainValue

        return self.setRegister(NAU7802_CTRL1, value)

    def setLDO(self, ldoValue: int):
        """Set the onboard Low-Drop-Out voltage regulator to a given value. 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available

        Args:
            ldoValue (int): [description]
        """
        if ldoValue > 0b111:
            ldoValue = 0b111

        value = self.getRegister(NAU7802_CTRL1)
        value &= 0b11000111
        value |= ldoValue << 3
        self.setRegister(NAU7802_CTRL1, value)

        return self.setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL)

    def setSampleRate(self, rate: int):
        """Set the readings per second. 10, 20, 40, 80, and 320 samples per second is available

        Args:
            rate (int): [description]
        """
        if rate > 0b111:
            rate = 0b111

        value = self.getRegister(NAU7802_CTRL2)
        value &= 0b10001111
        value |= rate << 4
        return self.setRegister(NAU7802_CTRL2, value)

    def setChannel(self, channelNumber: int):
        """Select between 1 and 2

        Args:
            channelNumber (int): [description]
        """
        if channelNumber == NAU7802_CHANNEL_1:
            return self.clearBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)
        else:
            return self.setBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)

    def calibrateAFE(self):
        """Synchronous calibration of the analog front end of the NAU7802. Returns true if CAL_ERR bit is 0 (no error)
        """
        self.beginCalibrateAFE()
        return self.waitForCalibrateAFE(1000)

    def beginCalibrateAFE(self):
        """Begin asynchronous calibration of the analog front end of the NAU7802. Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE().
        """
        self.setBit(NAU7802_CTRL2_CALS,
                    NAU7802_CTRL2)

    def waitForCalibrateAFE(self, timeout_ms: int = 0):
        """Wait for asynchronous AFE calibration to complete with optional timeout.

        Args:
            timeout_ms (int, optional): [description]. Defaults to 0.
        """
        begin = time.monotonic_ns() // NS_PER_MS
        cal_ready = self.calAFEStatus()

        while cal_ready == NAU7802_CAL_IN_PROGRESS:
            if ((timeout_ms > 0) and ((time.monotonic_ns() // NS_PER_MS) - begin > timeout_ms)):
                break
            time.sleep(0.001)
            cal_ready = self.calAFEStatus()

        if cal_ready == NAU7802_CAL_SUCCESS:
            return True
        return False

    def calAFEStatus(self):
        """Check calibration status.
        """
        if self.getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2):
            return NAU7802_CAL_IN_PROGRESS

        if self.getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2):
            return NAU7802_CAL_FAILURE

        return NAU7802_CAL_SUCCESS

    def reset(self):
        """Resets all registers to Power Of Defaults
        """
        self.setBit(NAU7802_PU_CTRL_RR,
                    NAU7802_PU_CTRL)
        time.sleep(0.001)
        return self.clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)

    def powerUp(self):
        """Power up digital and analog sections of scale, ~2mA
        """
        self.setBit(NAU7802_PU_CTRL_PUD,
                    NAU7802_PU_CTRL)
        self.setBit(NAU7802_PU_CTRL_PUA,
                    NAU7802_PU_CTRL)

        counter = 0
        while True:
            if self.getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL):
                break
            time.sleep(0.001)
            counter += 1
            if counter > 100:
                return False
        return True

    def powerDown(self):
        """Puts scale into low-power 200nA mode
        """
        self.clearBit(NAU7802_PU_CTRL_PUD,
                      NAU7802_PU_CTRL)
        return self.clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL)

    def setIntPolarityHigh(self):
        """Set Int pin to be high when data is ready(default)
        """
        return self.clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)

    def setIntPolarityLow(self):
        """Set Int pin to be low when data is ready
        """
        return self.setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)

    def getRevisionCode(self):
        """Get the revision code of this IC. Always 0x0F.
        """
        revisionCode = self.getRegister(NAU7802_DEVICE_REV)
        return revisionCode & 0x0F

    def setBit(self, bitNumber: int, registerAddress: int):
        """Mask & set a given bit within a register

        Args:
            bitNumber (int): [description]
            registerAddress (int): [description]
        """
        value = self.getRegister(registerAddress)
        value |= 1 << bitNumber
        return self.setRegister(registerAddress, value)

    def clearBit(self, bitNumber: int, registerAddress: int):
        """Mask & clear a given bit within a register

        Args:
            bitNumber (int): [description]
            registerAddress (int): [description]
        """
        value = self.getRegister(registerAddress)
        value &= ~(1 << bitNumber) & 0xFF
        return self.setRegister(registerAddress, value)

    def getBit(self, bitNumber: int, registerAddress: int):
        """Return a given bit within a register

        Args:
            bitNumber (int): [description]
            registerAddress (int): [description]
        """
        value = self.getRegister(registerAddress)
        value &= 1 << bitNumber
        return bool(value)

    def getRegister(self, registerAddress: int):
        """Get contents of a register

        Args:
            registerAddress (int): [description]
        """
        try:
            self.i2c.writeto(NAU7802_DeviceAddress, bytearray(
                [registerAddress]))

            read_buffer = bytearray(1)
            self.i2c.readfrom_into(NAU7802_DeviceAddress, read_buffer)
            return read_buffer[0]
        except OSError:
            return -1

    def setRegister(self, registerAddress: int, value: int):
        """Send a given value to be written to given address. Return true if successful

        Args:
            registerAddress (int): [description]
            value (int): [description]
        """
        try:
            self.i2c.writeto(NAU7802_DeviceAddress, bytearray(
                [registerAddress, value]))
            return True
        except OSError:
            return False
