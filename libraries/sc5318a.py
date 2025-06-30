
# -*- coding: utf-8 -*-
"""
Added upon by Bridgette McAllister, SignalCore 9/2/2022
adapted to SC5318A
---
Created on Fri Mar 19 14:11:31 2021
Adapted from Chao Zhou
---
A simple driver for SignalCore SC5521A to be used with QCoDes, transferred from the one written by Erick Brindock
"""

import ctypes
from ctypes import CDLL
from ctypes.wintypes import HANDLE
from typing import Any, Dict, Optional


#definitions for the register dictionary
INITIALIZE = 0x01   #Initialize the devices
SYSTEM_ACTIVE = 0x02    #Set the system active light
SYNTH_MODE = 0x03   #Set the synthesizer mode

RF_FREQUENCY = 0x10 #Set the frequency
IF_FREQUENCY = 0x11 #Set the IF frequency
LO_FREQUENCY = 0x12 #Enable RF output
LO_SOURCE = 0x13    #Configure LO source behavior
RF_AMP = 0x14   #Turn on the RF Amp (down converters only)
ATTENUATOR = 0x15   #Sets the attenuator
SIGNAL_PATH = 0x16  #Sets the path route, and spectral inversion
RESERVED1 = 0x17    #
STORE_DEFAULT_STATE = 0x18  #Save current settings as default
DEVICE_STANDBY = 0x19   #Set the device into power standby
REFERENCE_CLOCK = 0x1A  #Reference settings
REFERENCE_DAC_WORD = 0x1B   #Sets the reference DAC for freq adjustment
USER_EEPROM_WRITE = 0x1C    #Route LO source from internal or external LO input
AUTO_CALC_GAIN = 0x1D   #Store parameters to auto set attenuators to achieve gain
SYNTH_SELF_CAL = 0x1F   #Set up the frequency plan frequencies such as IF, RF

GET_DEVICE_PARAM = 0x30 #Get the current frequency
GET_TEMPERATURE = 0x31  #Load sensor temperature into the SPI output buffer
GET_DEVICE_STATUS = 0x32    #Load the board status into the SPI output buffer
GET_DEVICE_INFO =0x33   #Load the device info
CAL_EEPROM_READ = 0x34  #Read EEPROM 8 bytes (u64 read)
USER_EEPROM_READ = 0x35 #SPI out buffer into the USART transmit register
SERIAL_OUT_BUFFER = 0x36    #transfer user EEPROM data to SPI output buffer

EEPROM0 = 0 #EEPROM for factory and synthesizer cal 
EEPROM1 = 1 #EEPROM for conversion cal data

USEREEPROM = 1  
CALEEPROM = 0
CALEEPROMSIZE = 65536   #bytes
CALDATASTARTADD = 0x298 #start position for cal data on eeprom
CALDATALEN = 0x5580 #total byte length of cal data
RFCALFREQLEN = 83   #frequency point 6000MHz to 2650 MHz step 250 MHz
CALATTENSTEPS = 30  #attenuation steps 1-30 dB
IFCALFREQLEN = 35   #freq points over the bandwidth 100MHz to 3500MHz, step 100 MHz
TEMPCOLEN = 6   #3 set of 2 2nd order coefficients, freq<12 GHz, 12GHz<freq<20 GHz,
BYPASSFREQLEN = 61  #100 MHz to 6100 MHz, step 100 MHz
CALTEMP_ADD = 0x298 
TEMPCOEFF_ADD = 0x29C
IFCALFREQ_ADD = 0x398
IFRELGAIN_ADD = 0x4B0
IFATTENCAL_ADD = 0x5C8
BPCALFREQ_ADD = 0x6B8
BPABSGAIN_ADD = 0x7A8
RFCALFREQ_ADD = 0x898
RFNONINVGAIN_ADD = 0xBD0
RFINVGAIN_ADD = 0xF08
RFAMPGAIN_ADD = 0x1240
RFATTENCAL_ADD = 0x1578

DEVINFOSNINT = 0x00 #serial number address and interface
DEVINFOREV = 0x01   #hardware and firmware revision address
DEVINFODATE = 0x02  #manufactured and calibration date address

RFATTEN = 0
IFATTEN = 1


#definitions for the Signalcore device
MAXDEVICES = 50
MAXDESCRIPTORSIZE = 9

#define the comm interface being used
COMMINTERFACE = ctypes.c_uint(1)

class Signal_Path_Params_t(ctypes.Structure):
    _fields_ = [("bypass_converter", ctypes.c_uint8),
                ("rf_amp_enable", ctypes.c_uint8),
                ("if_out_enable", ctypes.c_uint8),
                ("invert_spectrum", ctypes.c_uint8)]
signal_path_params_t = Signal_Path_Params_t()

class Attenuator_t(ctypes.Structure):
    _fields_ = [("rf_atten_value", ctypes.c_float), #RF attenuation value
                ("if_atten_value", ctypes.c_float)] #IF attenuation value
attenuator_t = Attenuator_t()

class Device_rf_params_t(ctypes.Structure):
    _fields_ = [("rf_frequency", ctypes.c_double),  #RF port frequency
                ("if_frequency", ctypes.c_double),  #IF port frequency
                ("lo_frequency", ctypes.c_double),  #LO frequency
                ("atten", Attenuator_t),
                ("path_params", Signal_Path_Params_t)
                ]
device_rf_params = Device_rf_params_t()

class byte_data_t(ctypes.Structure):
    _fields_ = [("byte_data", ctypes.c_ubyte)]
byte_data = byte_data_t()

class cal_data_p(ctypes.Structure):
    _fields_ = [("calData_p", ctypes.c_void_p)]

class Device_temperature_t(ctypes.Structure):
    _fields_ = [("device_temp", ctypes.c_float)]

class Conversion_gain_t(ctypes.Structure):
    _fields_ = [("conversion_gain", ctypes.c_float)]

class Dac_value_t(ctypes.Structure):
    _fields_ = [("dac_value", ctypes.c_ushort)]

class Reg_read_t(ctypes.Structure):
    _fields_ = [("reg_read", ctypes.c_ulonglong)]
 
class Operate_status_t(ctypes.Structure):
    _fields_ = [("device_accessed", ctypes.c_uint8),    #indicates that the device is opened and accessed
                ("ext_ref_detected", ctypes.c_uint8),   #indicates that a suitable external reference is detected
                ("lock_ext_ref_enable", ctypes.c_uint8),    #indicates lock state of device to external source
                ("lo_supply_enable", ctypes.c_uint8),   #indicates LO power status
                ("ext_lo_enable", ctypes.c_uint8),  #LO drive select: 1 = internal, 0 = external
                ("ext_lo_path", ctypes.c_uint8),    #External LO input select 0 = rear LO input, 1 = front LO input
                ("lo_mode", ctypes.c_uint8),    #mode of computing the LO frequency. 0 = RF, IF, inversion 1 = LO direct
                ("lo_doubler_enable", ctypes.c_uint8),  #LO frequency x2 multiplier
                ("device_standby", ctypes.c_uint8), #signal path power state
                ("bypass_conv", ctypes.c_uint8),    #RF-in to RF-out port direct, no conversion
                ("if_out_enable", ctypes.c_uint8),  #IF output on/off
                ("invert_spect", ctypes.c_uint8),   #Converted spectrum polarity
                ("rf_amp_enable", ctypes.c_uint8),  #RF amplifier on/off
                ("auto_gain_enable", ctypes.c_uint8),   #Device automatically adjusts the atten and preamp to set gain of device close to the desired.
                                                        #This feature is disabled by default. Use the math based algorithms to set gain more precisely
                ("auto_amp_enable", ctypes.c_uint8),    #Device automatically controls the status of the RF preamplifier
                ("pxi10_clk_enable", ctypes.c_uint8)]   #Status of the PXI clock outputl
operate_status_t = Operate_status_t()

class pllStatus_t(ctypes.Structure):
    _fields_ = [("sum_pll_ld", ctypes.c_uint8),     #lock status of main pll loop
                ("crs_pll_ld", ctypes.c_uint8),     #lock status of coarse offset pll loop (used only for hamonic mode)
                ("fine_pll_ld", ctypes.c_uint8),    #lock status of the dds tuned fine pll loop
                ("vcxo_pll_ld", ctypes.c_uint8),    #lock status of the 100 MHz VCXO pll loop
                ("tcxo_pll_ld", ctypes.c_uint8),    #lock status of the master 10 MHz TCX) pll loop
                ("loop_gain", ctypes.c_uint8)]      #loop gain
pll_status_t = pllStatus_t()

class Device_status_t(ctypes.Structure):
    _fields_ = [("pll_status", pllStatus_t),    #contains the PLL status structure
                ("operate_status", Operate_status_t)]   #contains the Operate Status structure
device_status_t = Device_status_t()

class Date_t(ctypes.Structure):
    _fields_ = [("year", ctypes.c_uint16),
                ("month", ctypes.c_uint8),
                ("day", ctypes.c_uint8)]
date_t = Date_t()

class cal_data_t(ctypes.Structure):
    _fields_ = [("cal_temp", ctypes.c_float),   
                ("temp_coeff", ctypes.c_void_p),    #1D array of 2 coefficients c1, c2, 3 places
                ("if_cal_freq", ctypes.c_void_p),   #1D array of IF cal frequencies (MHz) 100 MHz to 5 GHz 100 MHz step
                ("if_rel_gain_cal", ctypes.c_void_p),   #relative gain to 2 GHz center with non inverting spectrum
                ("if_atten_cal", ctypes.c_void_p),  #1D array of relative IF atten cal, 1-30 dB, 1 dB step
                ("bp_rf_cal_freq", ctypes.c_void_p),    #1D array of bypass conversion frequencyes 250 MHz to 40 GHz, 250 MHz step
                ("bp_abs_gain_cal", ctypes.c_void_p),   #1D array Bypass abs gain at each freq
                ("rf_cal_freq", ctypes.c_void_p),   #1D array of rf cal frequencies 20 GHz to 40 GHz step 250 MHz
                ("rf_usb_abs_gain_cal", ctypes.c_void_p),   #1D array of rf abs gain at every rf freq
                ("rf_lsb_abs_gain_cal", ctypes.c_void_p),   #1D array of rf abs gain at every rf freq
                ("rf_amp_gain_cal", ctypes.c_void_p),   #1D array of relative RF amp cal at RF freq
                ("rf_atten_cal", ctypes.POINTER(ctypes.c_void_p))]  #2D array (attenxfreq) of RF atten 1 cal
cal_data = cal_data_t()

class Device_info_t(ctypes.Structure):
    _fields_ = [("product_sn", ctypes.c_uint32),
                ("fw_rev", ctypes.c_float),
                ("hw_rev", ctypes.c_float),
                ("dev_interface", ctypes.c_uint8),  #0=unassigned, 1 = PXI, 2 = USB and SPI, 3 = USB and RS232
                ("man_date", Date_t),
                ("cal_date", Date_t)
                ]
device_info_t = Device_info_t()

class Hw_trigger_t(ctypes.Structure):
    _fields_ = [("edge", ctypes.c_ubyte),
                ("pxi_enable", ctypes.c_ubyte),
                ("pxi_line", ctypes.c_ubyte)]
hw_trigger_t = Hw_trigger_t()

class Gain_params_t(ctypes.Structure):
    _fields_ = [("rf_level", ctypes.c_float),   #expected RF level input
                ("mixer_level", ctypes.c_float),    #max desired level at mixer input
                ("if_level", ctypes.c_float),   #expected if level output
                ("linear_mode", ctypes.c_ubyte),    #linear modes: 0 = mixer level control, 1 = balance of snr and linearity, 2 = better snr, 3= best snr, 4 = better linearity, 5 = best linearity
                ("auto_amp_ctrl", ctypes.c_ubyte)]  #allows the preamplifier state to be changed to achieve the desired results
gain_params_t = Gain_params_t()

class Synth_mode_t(ctypes.Structure):
    _fields_ = [("lock_mode", ctypes.c_uint8),
                ("loop_gain", ctypes.c_uint8),
                ("auto_spur_suppress", ctypes.c_uint8)]
synth_mode_t = Synth_mode_t()

# End of Structures------------------------------------------------------------     
 
 
def getdict(struct):
    """
    This is copied from online: 
    https://stackoverflow.com/questions/3789372/python-can-we-convert-a-ctypes-structure-to-a-dictionary
    """
    result = {}
    for field, _ in struct._fields_:
         value = getattr(struct, field)
         # if the type is not a primitive and it evaluates to False ...
         if (type(value) not in [int, float, bool]) and not bool(value):
             # it's a null pointer
             value = None
         elif hasattr(value, "_length_") and hasattr(value, "_type_"):
             # Probably an array
             value = list(value)
         elif hasattr(value, "_fields_"):
             # Probably another struct
             value = getdict(value)
         result[field] = value
    return result


class SignalCore_SC5318A():
    def __init__(self, name: str, serial_number: str, dll = None, debug = False, **kwargs: Any):

        super().__init__()
        self._serial_number = ctypes.c_char_p(bytes(serial_number, 'utf-8'))
        self._devices_number = ctypes.c_uint()
        buffers = [ctypes.create_string_buffer(MAXDESCRIPTORSIZE + 1) for bid in range (MAXDEVICES)]
        self.buffer_pointer_array = (ctypes.c_char_p * MAXDEVICES)()
        for device in range(MAXDEVICES):
            self.buffer_pointer_array[device]= ctypes.cast(buffers[device], ctypes.c_char_p)

        self._buffer_pointer_array_p = ctypes.cast(self.buffer_pointer_array, ctypes.POINTER(ctypes.c_char_p))
        
        if dll is not None:
            self._dll = dll
        else:
            self._dll = ctypes.WinDLL('C:\\Program Files\\SignalCore\\SC5317A_18A\\api\\c\\lib\\x64\\mj2.dll')

        found = self._dll.mj2_SearchDevices(COMMINTERFACE, self._buffer_pointer_array_p, ctypes.byref(self._devices_number))
        if found:
            print ("Failed to find any device")

        lib = ctypes.WinDLL('C:\\Program Files\\SignalCore\\SC5317A_18A\\api\\c\\lib\\x64\\mj2.dll')

        self._byte_data = byte_data_t(0)
        self._signal_path_params_t = Signal_Path_Params_t(0, 0, 0, 0)
        self._attenuator_t = Attenuator_t(0.0, 0.0)        
        self._rf_params = Device_rf_params_t(0, 0, 0, self._attenuator_t, self._signal_path_params_t)
        self._status = Operate_status_t(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

        self._conversion_gain = Conversion_gain_t(0)
        self._temperature = Device_temperature_t(0)
        self._dac_value = Dac_value_t(0)
        self._reg_read = Reg_read_t(0)
        self._status = Operate_status_t()
        self._pll_status_t = pllStatus_t()
        self._device_status = Device_status_t(self._pll_status_t, self._status)
        self._date_t = Date_t(0, 0, 0)
        self._cal_data = cal_data_t()
        self._device_info_t = Device_info_t(0, 0, 0, 0, self._date_t, self._date_t)
        self._hw_trigger_t = Hw_trigger_t(0, 0, 0)
        self._gain_params_t = Gain_params_t(0, 0, 0, 0, 0)

        self._open()

    def _open(self) -> None:
        """ Opens a device session"""
        self._handle = HANDLE() #ctypes.wintypes.HANDLE()
        self._dll.mj2_OpenDevice(COMMINTERFACE, self._serial_number, 0, ctypes.byref(self._handle))

    def init(self, state = 0):
        error_code = self._dll.mj2_InitDevice(self._handle, ctypes.c_uint(state))
        return error_code, state

    def _close(self) -> None:
        """ Closes the device session """
        self._dll.mj2_CloseDevice(self._handle)

    def set_atten(self, atten_type = 0, attenuation = 0.0):
        """ Sets the RF attenuation of the device 
            input: float"""
        error_code = self._dll.mj2_SetAttenuator(self._handle, ctypes.c_uint(atten_type), ctypes.c_float(attenuation))
        return error_code, attenuation

    def set_standby(self, standby_enable = False):
        """Sets the standby of the device.
            Input: 
                standby (bool) True = standby enabled, False = standby disabled."""

        error_code = self._dll.mj2_SetStandby(self._handle, ctypes.c_uint(standby_enable))
        return error_code, standby_enable

    def set_frequency(self, frequency: float):
        """
        Sets RF1 frequency. Valid between 100MHz and 20GHz
            Input:
                frequency (int) = frequency in Hz
        """
        error_code = self._dll.mj2_SetFrequency(self._handle, ctypes.c_double(frequency))
        return error_code, frequency

    def set_lo_frequency(self, lo_frequency: float):
        error_code = self._dll.mj2_SetLoFrequency(self._handle, ctypes.c_double(lo_frequency))
        return error_code, lo_frequency

    def set_if_frequency(self, if_frequency: float):
        error_code = self._dll.mj2_SetIfFrequency(self._handle, ctypes.c_double(if_frequency))
        return error_code, if_frequency

    def get_frequency(self):
        """
        Gets frequency in Hz
        """
        error_code = self._dll.mj2_GetRfParameters(self._handle, ctypes.byref(self._rf_params))
        frequency_status = self._rf_params.rf_params
        return error_code, frequency_status

    def store_default_state(self):
        """Stores the default state of the device"""
        error_code = self._dll.mj2_SetAsDefault(self._handle)
        return error_code

    def get_temperature(self) -> float:
        """Returns the current device temperature as a float value"""
        error_code= self._dll.mj2_GetTemperature(self._handle, ctypes.byref(self._temperature))
        temp = self._temperature.device_temp
        return temp, error_code

    def get_gain_params(self):
        """returns all of the gain parameters"""
        error_code = self._dll.mj2_GetGainCalcParams(self._handle, ctypes.byref(self._gain_params_t))
        GAIN : Dict[str, Optional[str]] = {
            'rf_level' : self._gain_params_t.rf_level,
            'mixer_level' : self._gain_params_t.mixer_level,
            'if_level' : self._gain_params_t.if_level,
            'linear_mode' : self._gain_params_t.linear_mode,
            'auto_amp_ctrl' : self._gain_params_t.auto_amp_ctrl}
        return error_code, GAIN
        
    def get_rf_parameters(self):
        """Returns all of the RF parameters"""
        error_code = self._dll.mj2_GetRfParameters(self._handle, ctypes.byref(self._rf_params))
        RF: Dict[str, Optional[str]] = {
            'vendor' : "SignalCore", 
            'model' : "SC5318A",
            'rf_frequency' : self._rf_params.rf_frequency,
            'if_frequency' : self._rf_params.if_frequency,
            'lo_frequency' : self._rf_params.lo_frequency,
            'rf_atten_value' : self._rf_params.atten.rf_atten_value,
            'if_atten_value' : self._rf_params.atten.if_atten_value,
            'bypass_conversion' : self._rf_params.path_params.bypass_converter,
            'rf_amp_enable' : self._rf_params.path_params.rf_amp_enable,
            'if_out_enable' : self._rf_params.path_params.if_out_enable,
            'invert_spectrum' : self._rf_params.path_params.invert_spectrum
            }

        return error_code, RF

    def get_device_idn(self)-> Dict[str, Optional[str]]:
        """Returns the device's identity"""
        error_code = self._dll.mj2_GetDeviceInfo(self._handle, ctypes.byref(self._device_info_t))
        IDN: Dict[str, Optional[str]] = {
            'vendor': "SignalCore",
            'model': "SC5318A",
            'serial_number': self._serial_number.value.decode("utf-8"),
            'firmware_revision': self._device_info_t.fw_rev,
            'hardware_revision': self._device_info_t.hw_rev,
            'device_interface' : self._device_info_t.dev_interface,
            'calibration_date' : '{}-{}-{}'.format(self._device_info_t.cal_date.year, self._device_info_t.cal_date.month, self._device_info_t.cal_date.day),
            'manufacture_year': '{}'.format(self._device_info_t.man_date.year),
            'manufacture_month' : '{}'.format(self._device_info_t.man_date.month),
            'manufacture_day' : '{}'.format(self._device_info_t.man_date.day)}
        return error_code, IDN

    def get_device_status(self)-> Dict[str, Optional[str]]:
        """Returns the device's operate status values"""
        error_code = self._dll.mj2_GetDeviceStatus(self._handle, ctypes.byref(self._device_status))
        device_status_t = self._device_status
        pll_status = self._pll_status_t
        operate_status = self._status
        PLL: Dict[str, Optional[str]] = {

            'sum_pll_ld' : device_status_t.pll_status.sum_pll_ld,
            'crs_pll_ld' : device_status_t.pll_status.crs_pll_ld,
            'fine_pll_ld' : device_status_t.pll_status.fine_pll_ld,
            'vcxo_pll_ld' : device_status_t.pll_status.vcxo_pll_ld,
            'tcxo_pll_ld' : device_status_t.pll_status.tcxo_pll_ld,
            'loop_gain' : device_status_t.pll_status.loop_gain}

        OPERATE: Dict[str, Optional[str]] = {
            'vendor' : "SignalCore",
            'model' : "SC5318A",
            'device_accessed' : device_status_t.operate_status.device_accessed,
            'ext_ref_detected' : device_status_t.operate_status.ext_ref_detected,
            'lock_ext_ref_enable' : device_status_t.operate_status.lock_ext_ref_enable,
            'lo_supply_enable' : device_status_t.operate_status.lo_supply_enable,
            'ext_lo_enable' : device_status_t.operate_status.ext_lo_enable,
            'ext_lo_path' : device_status_t.operate_status.ext_lo_path,
            'lo_mode' : device_status_t.operate_status.lo_mode,
            'lo_doubler_enable' : device_status_t.operate_status.lo_doubler_enable,
            'device_standby' : device_status_t.operate_status.device_standby,
            'bypass_conv' : device_status_t.operate_status.bypass_conv,
            'if_out_enable' : device_status_t.operate_status.if_out_enable,
            'invert_spect' : device_status_t.operate_status.invert_spect,
            'rf_amp_enable' : device_status_t.operate_status.rf_amp_enable,
            'auto_gain_enable' : device_status_t.operate_status.auto_gain_enable,
            'auto_amp_enable' : device_status_t.operate_status.auto_amp_enable,
            'pxi10_clk_enable' : device_status_t.operate_status.pxi10_clk_enable}


        return error_code, PLL, OPERATE

    def reg_write(self, reg_byte, inst_word = int):
        """Writes the specific value to the register, see documentation for more info"""
        instruct_word = ctypes.c_ulonglong(inst_word)
        error_code = self._dll.mj2_RegWrite(self._handle, reg_byte, instruct_word)
        return error_code, inst_word

    def reg_read(self, reg_byte, ins_word = 0):
        """Reads back the specific value back, see documentation for more info"""
        reg_byte = ctypes.c_ubyte(reg_byte)
        instruct_word = ctypes.c_ulonglong(ins_word)
        error_code = self._dll.mj2_RegRead(self._handle, reg_byte, instruct_word, ctypes.byref(self._reg_read))
        rec_word = self._reg_read.reg_read
        return error_code, rec_word, ins_word

    def set_synth_mode(self, pll_loop_gain = 0):
        """
        sets the rf mode of the device
        Disable spur suppress: (only takes effect when lock mode is harmonic)
            input: integer = 0 = not enabled, 1 = spur suppress by lowering loop gain and/or ping ponging between lock modes automatically
        Low loop gain:
            input: integer = 0 = Normal gain, 1 = low gain
        Lock mode:
            input: integer = 0 = harmonic, 1 = fractional
        """

        error_code = self._dll.mj2_SetSynthMode(self._handle, ctypes.c_uint(pll_loop_gain))
        return error_code, pll_loop_gain

    def set_reference_dac(self, dac_value = 0):
        """Sets a value to the reference dac
        input: integer"""
        error_code = self._dll.mj2_SetReferenceDac(self._handle, ctypes.c_uint(dac_value))
        return error_code, dac_value

    def synth_self_cal(self):
        """Tells the device to perform a synthesizer self calibration"""
        error_code = self._dll.mj2_SetSynthSelfCal(self._handle)
        return error_code

    def set_preamp(self, preAmpStatus = 0):
        error_code = self._dll.mj2_SetPreamp(self._handle, ctypes.c_uint(preAmpStatus))
        return error_code, preAmpStatus

    def set_lo_source(self, loSelect = 0, loMode = 0, loDoubler = 0, extLoPath = 0):
        error_code = self._dll.mj2_SetLoSource(self._handle, ctypes.c_uint(loSelect), ctypes.c_uint(loMode), ctypes.c_uint(loDoubler), ctypes.c_uint(extLoPath))
        return error_code, loSelect, loMode, loDoubler, extLoPath

    def set_signal_path(self, bypassConverter = 0, rfAmpEnable = 0, ifOutEnable = 0, invertSpectrum = 0):
        sp = Signal_Path_Params_t(bypass_converter = bypass_converter, rf_amp_enable = rf_amp_enable, if_out_enable = if_out_enable, invert_spectrume = invert_spectrum)
        error_code = self._dll.mj2_SetSignalPath(self._handle, ctypes.byref(sp))
        return error_code, sp

    def calc_gain(self):
        """Calculates the gain value of the device at a specified frequency.
        Args:

        Returns:
        The gain value as a float, and the error code as an integer.
        """
        error_code = self._dll.mj2_CalcGain(self._handle, ctypes.byref(self._conversion_gain))
        gain = self._conversion_gain.conversion_gain
        return error_code, gain

    def set_reference_clock(self, lock_ext_enable = 0):
        error_code = self._dll.mj2_SetReferenceClock(self._handle, ctypes.c_uint(lock_ext_enable))
        return error_code, lock_ext_enable

    def write_user_eeprom(self, memAdd = 0, byte_data = 0):
        error_code = self._dll.mj2_WriteUserEeprom(self._handle, ctypes.c_uint(memAdd), ctypes.c_uint(byte_data))
        return error_code, memAdd, byte_data

    def read_eeprom(self, eeprom_sel = 0, mem_start_add = 0, len = 0):
        eeprom_sel = ctypes.c_uint(eeprom_sel)
        mem_start_add = ctypes.c_uint(mem_start_add)
        byte_array = ctypes.c_ubyte*len

        array=byte_array()

        error_code = self._dll.mj2adv_ReadEeprom(self._handle, eeprom_sel, mem_start_add, ctypes.c_uint32(len), array)

        return error_code, eeprom_sel, mem_start_add, len, array
