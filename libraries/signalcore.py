#READ THE FIRST PART OF THE README.TXT FILE BEFORE RUNNING
#Thus far, this code can only support one device

#calling the sc5318a lib of functions and definitions
import os
import sys
import sc5318a
import time
from sc5318a import *


error_dict = {'0' : 'SCI_SUCCESS',
              '0' : 'SCI_ERROR_NONE',
              '-1' : 'SCI_ERROR_INVALID_DEVICE_HANDLE',
              '-2' : 'SCI_ERROR_NO_DEVICE',
              '-3' : 'SCI_ERROR_INVALID_DEVICE',
              '-4' : 'SCI_ERROR_MEM_UNALLOCATE',
              '-5' : 'SCI_ERROR_MEM_EXCEEDED',
              '-6' : 'SCI_ERROR_INVALID_REG',
              '-7' : 'SCI_ERROR_INVALID_ARGUMENT',
              '-8' : 'SCI_ERROR_COMM_FAIL',
              '-9' : 'SCI_ERROR_OUT_OF_RANGE',
              '-10' : 'SCI_ERROR_PLL_LOCK',
              '-11' : 'SCI_ERROR_TIMED_OUT',
              '-12' : 'SCI_ERROR_COMM_INIT',
              '-13' : 'SCI_ERROR_TIMED_OUT_READ'
    }

def _error_handler(msg: int) -> None:
        """Display error when setting the device fails

        Args: 
            msg(int): error key, see error_dict dict.
        Raises:
            BaseException
        """

        if msg!=0:
            raise BaseException("Couldn't set the device due to {}.".format(error_dict[str(msg)]))
        else:
            pass

def device_temp():
    #fetches the current temperature of the device
        temp, error_code = SC1.get_temperature()
        _error_handler(error_code)
        print(temp)
        return temp

def get_rf_frequency():
    #fetches the current RF frequency of the device
    error_code, RF = SC1.get_rf_parameters()
    frequency = RF['rf_frequency']
    print("The frequency is : {:.2f}".format(frequency))
    _error_handler(error_code)
    return frequency

def get_if_frequency():
    #fetches the current IF frequency of the device
    error_code, RF = SC1.get_rf_parameters()
    if_frequency = RF['if_frequency']
    print("The IF frequency is : {:.2f}".format(if_frequency))
    _error_handler(error_code)
    return if_frequency

def get_lo_frequency():
    #fetches the current LO frequency of the device
    error_code, RF = SC1.get_rf_parameters()
    lo_frequency = RF['lo_frequency']
    print("The LO frequency is : {:.2f}".format(lo_frequency))
    _error_handler(error_code)
    return lo_frequency

def get_rf_atten_value():
    #fetches the current RF attenuation value
    error_code, RF = SC1.get_rf_parameters()
    rf_atten = RF['rf_atten_value']
    print("The RF Attenuation is : {:.2f}".format(rf_atten))
    _error_handler(error_code)
    return rf_atten

def get_if_atten_value():
    #fetches the current IF attenuation value
    error_code, RF = SC1.get_rf_parameters()
    if_atten = RF['if_atten_value']
    print("The IF Attenuation is : {:.2f}".format(if_atten))
    _error_handler(error_code)
    return if_atten

def get_bypass_conversion():
    #returns if the bypass conversion is enabled or disabled
    error_code, RF = SC1.get_rf_parameters()
    bypass = RF['bypass_conversion']
    if RF['bypass_conversion'] == 0:
        print("The bypass conversion is disabled")
    if RF['bypass_conversion'] == 1:
        print("The bypass conversion is enabled")
    _error_handler(error_code)
    return bypass

def get_rf_amp_enable():
    #returns if the RF amplifier is enabled or disabled
    error_code, RF = SC1.get_rf_parameters()
    rf_amp = RF['rf_amp_enable']
    if RF['rf_amp_enable'] == 0:
        print("The RF amplifier is disabled")
    if RF['rf_amp_enable'] == 1:
        print("The RF amplifier is enabled")
    _error_handler(error_code)
    return rf_amp

def get_device_access():
    #returns if the device has been accessed or not
    error_code, PLL, OPERATE = SC1.get_device_status()
    access = OPERATE['device_accessed']
    if OPERATE['device_accessed'] == 0:
        print("The device has not been accessed")
    if OPERATE['device_accessed'] == 1:
        print("The device has been accessed")
    _error_handler(error_code)
    return access

def get_ext_ref_detected():
    #returns if an external reference has been detected, or not
    error_code, PLL, OPERATE = SC1.get_device_status()
    ext_ref_detect = OPERATE['ext_ref_detected']
    if OPERATE['ext_ref_detected'] == 0:
        print("No external reference has been detected")
    if OPERATE['ext_ref_detected'] == 1:
        print("An external reference has been detected")
    _error_handler(error_code)
    return ext_ref_detect

def get_lock_ext_ref_enable():
    #returns if the lock to the external reference is disabled or enabled
    error_code, PLL, OPERATE = SC1.get_device_status()
    ext_ref = OPERATE['lock_ext_ref_enable']
    if OPERATE['lock_ext_ref_enable'] == 0:
        print("Lock to external reference is disabled")
    if OPERATE['lock_ext_ref_enable'] == 1:
        print("Lock to external reference is enabled")
    _error_handler(error_code)
    return ext_ref

def get_lo_pwr_enable():
    #returns if the LO power enable is disabled or enabled
    error_code, PLL, OPERATE = SC1.get_device_status()
    lo_pwr = OPERATE['lo_supply_enable']
    if OPERATE['lo_supply_enable'] == 0:
        print("The LO power enable is disabled")
    if OPERATE['lo_supply_enable'] == 1:
        print("The LO power enable is enabled")
    _error_handler(error_code)
    return lo_pwr

def get_ext_lo_enable():
    #returns if the LO selection is internal or external
    error_code, PLL, OPERATE = SC1.get_device_status()
    ext_lo_enable = OPERATE['ext_lo_enable']
    if OPERATE['ext_lo_enable'] == 0:
        print("The LO selection is internal")
    if OPERATE['ext_lo_enable'] == 1:
        print("The LO selection is external")
    _error_handler(error_code)
    return ext_lo_enable

def get_ext_lo_in_select():
    #returns if the external LO input selection is in the rear or front LO input
    error_code, PLL, OPERATE = SC1.get_device_status()
    ext_lo_select = OPERATE['ext_lo_path']
    if OPERATE['ext_lo_path'] == 0:
        print("The external LO input selection is the rear LO input")
    if OPERATE['ext_lo_path'] == 1:
        print("The external LO input selection is the front LO input")
    _error_handler(error_code)
    return ext_lo_select

def get_lo_mode():
    #states if the LO uses LO or RF/IF
    error_code, PLL, OPERATE = SC1.get_device_status()
    lo_mode = OPERATE['lo_mode']
    if OPERATE['lo_mode'] == 0:
        print("The LO uses RF/IF")
    if OPERATE['lo_mode'] == 1:
        print("The LO uses the LO")
    _error_handler(error_code)
    return lo_mode

def get_lo_doubler_enable():
    #states if the LO frequency double is enabled or disabled
    error_code, PLL, OPERATE = SC1.get_device_status()
    lo_doubler = OPERATE['lo_doubler_enable']
    if OPERATE['lo_doubler_enable'] == 0:
        print("The LO frequency doubler is enabled")
    if OPERATE['lo_doubler_enable'] == 1:
        print("The LO frequency doubler is disabled")
    _error_handler(error_code)
    return lo_doubler

def get_device_standby():
    #states if the device is in standby mode or not
    error_code, PLL, OPERATE = SC1.get_device_status()
    standby = OPERATE['device_standby']
    if OPERATE['device_standby'] == 0:
        print("The device is not on standby")
    if OPERATE['device_standby'] == 1:
        print("The device is on standby")
    _error_handler(error_code)
    return standby

def get_auto_gain_enable():
    #states if the automatic gain is disabled or enabled
    error_code, PLL, OPERATE = SC1.get_device_status()
    auto_gain = OPERATE['auto_gain_enable']
    if OPERATE['auto_gain_enable'] == 0:
        print("The Automatic gain is disabled")
    if OPERATE['auto_gain_enable'] == 1:
        print("The Automatic gain is enabled")
    _error_handler(error_code)
    return auto_gain

def get_auto_amp_enable():
    #states if the automatic amplifier is enabled or disabled
    error_code, PLL, OPERATE = SC1.get_device_status()
    auto_amp = OPERATE['auto_amp_enable']
    if OPERATE['auto_amp_enable'] == 0:
        print("The automatic amplifier is disabled")
    if OPERATE['auto_amp_enable'] == 1:
        print("The automatic amplifier is enabled")
    _error_handler(error_code)
    return auto_amp

def get_pxi_10_clk_enable():
    #states if the PXI clock is enabled or disabled
    error_code, PLL, OPERATE = SC1.get_device_status()
    pxi_clk = OPERATE['pxi10_clk_enable']
    if OPERATE['pxi10_clk_enable'] == 0:
        print("The PXI clock is disabled")
    if OPERATE['pxi10_clk_enable'] == 1:
        print("The PXI clock is enabled")
    _error_handler(error_code)
    return pxi_clk

def get_if_out_enable():
    #states if the IF out is enabled or disabled
    error_code, PLL, OPERATE = SC1.get_device_status()
    if_out = OPERATE['if_out_enable']
    if OPERATE['if_out_enable'] == 0:
        print("The IF out is disabled")
    if OPERATE['if_out_enable'] == 1:
        print("The IF out is enabled")
    _error_handler(error_code)
    return if_out

def get_invert_spectrum():
    #states if the invert spectrum option is enabled or disabled
    error_code, PLL, OPERATE = SC1.get_device_status()
    invert_spect = OPERATE['invert_spect']
    if OPERATE['invert_spect'] == 0:
        print("The 'invert spectrum' is not enabled")
    if OPERATE['invert_spect'] == 1:
        print("The 'invert spectrum' is enabled")
    _error_handler(error_code)
    return invert_spect

def get_rf_amp_enable():
    #states if the RF amplifier is disabled or enabled
    error_code, PLL, OPERATE = SC1.get_device_status()
    rf_amp = OPERATE['rf_amp_enable']
    if OPERATE['rf_amp_enable'] == 0:
        print("The RF amplifier is disabled")
    if OPERATE['rf_amp_enable'] == 1:
        print("The RF amplifier is enabled")
    _error_handler(error_code)
    return rf_amp


def get_lo_sum_pll_ld():
    #states if the LO sum PLL is unlocked or locked
    error_code, PLL, OPERATE = SC1.get_device_status()
    sum_pll = PLL['sum_pll_ld']
    if PLL['sum_pll_ld'] == 0:
        print("The LO sum PLL is unlocked")
    if PLL['sum_pll_ld'] == 1:
        print("The LO sum PLL is locked")
    _error_handler(error_code)
    return sum_pll

def get_lo_crs_pll_ld():
    #states if the LO coarse PLL is locked or unlocked
    error_code, PLL, OPERATE = SC1.get_device_status()
    lo_crs_pll = PLL['crs_pll_ld']
    if PLL['crs_pll_ld'] == 0:
        print("The LO coarse PLL is unlocked")
    if PLL['crs_pll_ld'] == 1:
        print("The LO coarse PLL is locked")
    _error_handler(error_code)
    return lo_crs_pll

def get_lo_fine_pll_ld():
    #states if the LO fine PLL is locked or unlocked
    error_code, PLL, OPERATE = SC1.get_device_status()
    lo_fine_pll = PLL['fine_pll_ld']
    if PLL['fine_pll_ld'] == 0:
        print("The LO fine PLL is unlocked")
    if PLL['fine_pll_ld'] == 1:
        print("The LO fine PLL is locked")
    _error_handler(error_code)
    return lo_fine_pll

def get_vcxo_pll_lock():
    #states if the VCXO PLL is locked or unlocked
    error_code, PLL, OPERATE = SC1.get_device_status()
    vcxo_pll = PLL['vcxo_pll_ld']
    if PLL['vcxo_pll_ld'] == 0:
        print("The VCXO PLL is unlocked")
    if PLL['vcxo_pll_ld'] == 1:
        print("The VCXO PLL is locked")
    _error_handler(error_code)
    return vcxo_pll

def get_tcxo_pll_lock():
    #states if the TCXO PLL is locked or unlocked
    error_code, PLL, OPERATE = SC1.get_device_status()
    tcxo_pll = PLL['tcxo_pll_ld']
    if PLL['tcxo_pll_ld'] == 0:
        print("The TCXO PLL is unlocked")
    if PLL['tcxo_pll_ld'] == 1:
        print("The TCXO PLL is locked")
    _error_handler(error_code)
    return tcxo_pll

def get_lo_loop_gain():
    #states if the LO loop gain is low, normal, or high
    error_code, PLL, OPERATE = SC1.get_device_status()
    loop_gain = PLL['loop_gain']
    if PLL['loop_gain'] == 0:
        print("The LO loop gain is low")
    if PLL['loop_gain'] == 1:
        print("The LO loop gain is normal")
    if PLL['loop_gain'] == 2:
        print("The LO loop gain is high")
    _error_handler(error_code)
    return loop_gain

def get_rf_level():
    #states what the RF level is
    error_code, GAIN = SC1.get_gain_params()
    rf_level = GAIN['rf_level']
    print("The RF level is : {:.2f}".format(rf_level))
    return rf_level

def get_mixer_level():
    #states what the mixer level is
    error_code, GAIN = SC1.get_gain_params()
    mixer_level = GAIN['mixer_level']
    print("The mixer level is : {:.2f}".format(mixer_level))
    _error_handler(error_code)
    return mixer_level

def get_if_level():
    #states what the IF level is
    error_code, GAIN = SC1.get_gain_params()
    if_level = GAIN['if_level']
    print("The IF level is : {:.2f}".format(if_level))
    _error_handler(error_code)
    return if_level

def get_linear_mode():
    #states what the linearity mode is
    error_code, GAIN = SC1.get_gain_params()
    linear_mode = GAIN['linear_mode']
    if GAIN['linear_mode'] == 0:
        print("The linearity mode is mixer level")
    if GAIN['linear_mode'] == 1:
        print("The linearity mode is noise-linearity balance")
    if GAIN['linear_mode'] == 2:
        print("The linearity mode is better SNR")
    if GAIN['linear_mode'] == 3:
        print("The linearity mode is best SNR")
    if GAIN['linear_mode'] == 4:
        print("The linearity mode is better linearity")
    if GAIN['linear_mode'] == 5:
        print("The linearity mode is best linearity")
    _error_handler(error_code)
    return linear_mode

def get_auto_ctrl_rf_amp():
    #states what the current status of the auto control RF amplifier
    error_code, GAIN = SC1.get_gain_params()
    auto_ctrl = GAIN['auto_amp_ctrl']
    if GAIN['auto_amp_ctrl'] == 0:
        print("The auto control RF amplifier is disabled")
    if GAIN['auto_amp_ctrl'] == 1:
        print("The auto control RF amplifier is enabled")
    _error_handler(error_code)
    return auto_ctrl

def get_idn(SC1):
    #returns the device's info
    error_code, IDN = SC1.get_device_idn()
    print(IDN)
    _error_handler(error_code)
    return IDN

def display_rf_parameters():
    #returns the RF parameter values
    error_code, RF = SC1.get_rf_parameters()
    print(RF)
    _error_handler(error_code)
    return 0

def display_operate_status():
    #returns the operate parameter values
    error_code, PLL, OPERATE = SC1.get_device_status()
    print(OPERATE)
    _error_handler(error_code)
    return 0

def display_pll_status():
    #returns the PLL parameter values
    error_code, PLL, OPERATE = SC1.get_device_status()
    print(PLL)
    _error_handler(error_code)
    return 0

def display_gain_status():
    #returns the gain parameter values
    error_code, GAIN = SC1.get_gain_params()
    print(GAIN)
    _error_handler(error_code)
    return 0

def close_device():
    SC1._close()

def set_rf_atten(atten=0.0):
    #sets the RF attenuator
    rf_type = 0
    error_code, attenuation = SC1.set_atten(rf_type, atten)
    _error_handler(error_code)
    return attenuation

def set_if_atten(atten=0.0):
    #sets the IF attenuator
    if_type = 1
    error_code, attenuation = SC1.set_atten(if_type, atten)
    _error_handler(error_code)
    return attenuation

def set_standby(standby_state):
    #Puts the device in standby mode
    #0: take device out of power standby
    #1: put the device into power standby
    error_code, standby = SC1.set_standby(standby_state)
    _error_handler(error_code)
    return standby

def set_frequency(freq_set):
    #sets the device RF port frequency
    error_code, frequency = SC1.set_frequency(freq_set)
    _error_handler(error_code)
    return frequency

def set_lo_frequency(lo_set):
    #sets the device LO frequency
    error_code, lo = SC1.set_lo_frequency(lo_set)
    _error_handler(error_code)
    return lo

def set_if_frequency(if_set):
    #sets the device IF frequency
    error_code, if_freq = SC1.set_if_frequency(if_set)
    _error_handler(error_code)
    return if_freq

def set_reference_dac(dac):
    #sets the value of the Clock reference DAC and hence adjusts reference clock frequency
    error_code, dac_value = SC1.set_reference_dac(dac)
    _error_handler(error_code)
    return dac_value

def synth_self_cal():
    #self cal of the synthesizer to align the VCOs
    error_code = SC1.synth_self_cal()
    _error_handler(error_code)
    return error_code

def reg_write(reg, inst):
    #reading the register via the USB device handle allocated by open_device
    #input: reg contains the target register address
    #input: inst contains necessary data for the specified register address
    error_code, inst_word = SC1.reg_write(reg, inst)
    _error_handler(error_code)
    return inst_word

def reg_read(reg, inst):
    #reading the register via the USB device handle allocated by open_device
    #input: reg contains the target register address
    #input: inst contains necessary data for the specified register address
    #output: rec_word is the return data request through the reg and inst
    error_code, rec_word, ins_word = SC1.reg_read(reg, inst)
    _error_handler(error_code)
    print(rec_word)
    return rec_word, ins_word

def set_preamp(preamp):
    #sets the RF preamplifier
    """
    Sets the RF preamplifier
    :param preamp: value to set the preamplifier
    :return: the status of the preamplifier
    """
    error_code, preAmpStatus = SC1.set_preamp(preamp)
    _error_handler(error_code)
    print(preAmpStatus)
    return preAmpStatus

def set_lo_source(select, mode, doubler, path):
    #sets the LO source
    """
    Sets the LO source configuration

    Parameters
    ----------
    select : int
        Selects the LO source, 0 = internal; 1 = external
    mode : int
        only activated if 'select' is 0
        0 = the LO frequency is calculated based on RF and IF. 
        1 = the LO frequency is directly programmed
    doubler : int
        Enables(1) or disables(0) the LO frequency doubler when 'select' is 1
    path : int
        Selects the external LO input port, 1 = rear LO input, 0 = front LO input

    Returns
    -------
    loSelect : int
        The LO source select
    loMode : int
        The LO mode
    loDoubler : int
        The LO doubler status
    extLoPath : int
        The external LO input path
    """

    error_code, loSelect, loMode, loDoubler, extLoPath = SC1.set_lo_source(select, mode, doubler, path)
    _error_handler(error_code)
    return loSelect, loMode, loDoubler, extLoPath

def set_synth_mode(loop_gain):
    #sets the synthesizer mode
    """
    Sets the synthesizer mode

    Parameters
    ----------
    loop_gain : int
        loop gain of the synthesizer, 0 = normal gain, 1 = low gain

    Returns
    -------
    pll_loop_gain : int
        The actual loop gain of the synthesizer
    """
    error_code, pll_loop_gain = SC1.set_synth_mode(loop_gain)
    _error_handler(error_code)
    return pll_loop_gain

def set_reference_clock(lock):
    #set the reference clock configurations
    #lock_ext_enable enables the device to lock to an external source
    #if the source not available error code returns. The device
    #will not attempt to lock and waits for source.
    """
    Set the reference clock configurations

    Parameters
    ----------
    lock : int
        enables the device to lock to an external source
        if the source not available error code returns. The device
        will not attempt to lock and waits for source.

    Returns
    -------
    lock_ext_enable : int
        The actual status of the reference clock lock
    """

    error_code, lock_ext_enable = SC1.set_reference_clock(lock)
    _error_handler(error_code)
    return lock_ext_enable

def set_as_default():
    #store the current state of the signal source into EEPROM as the default startup state
    error_code = SC1.store_default_state()
    _error_handler(error_code)
    return 0

def write_user_eeprom(mem, byte):
    #write single byte to the user EEPROM address
    #memAdd: the address of the EEPROM memory to write to
    #byte_data: the byte data to write to the specified memory address
    error_code, memAdd, byte_data = SC1.write_user_eeprom(mem, byte)
    _error_handler(error_code)
    return memAdd, byte_data

def to_hex(numbers):
    #takes an input and converts to hex
    if type(numbers) == int:
        numbers = [numbers]

    hex_str = "".join(["{:02X}".format(n) for n in numbers])
    return hex_str

def read_eeprom(sel, start, len):
    #reads from the eeprom
    error_code, eeprom_sel, mem_start_add, len, array = SC1.read_eeprom(sel, start, len)
    _error_handler(error_code)

    bytes_stored = []
    i=len

    for i in range (len, 0, -1):
        bytes_stored_temp = array[i-1]
        bytes_stored.append(bytes_stored_temp)
        hex_str = to_hex(bytes_stored)

    print(hex_str)
    return eeprom_sel, mem_start_add, len, array, hex_str

def calc_max_gain(max, preamp):
    #calculates the maximum of the device for the RF frequency and IF frequency chosen
    error_code, max_gain, pre_amp_gain = SC1.calc_max_gain(max, preamp)
    _error_handler(error_code)
    print(max)
    return max, preamp

def set_signal_path(bypass, rf_amp, if_out, invert):
    #set the signal path route of the converter
    error_code, sp = SC1.set_signal_path(bypass, rf_amp, if_out, invert)
    _error_handler(error_code)
    return bypass, rf_amp, if_out, invert

def calc_gain():
    #function calculates the conversion gain of the device with its current setup
    error_code, gain = SC1.calc_gain()
    _error_handler(error_code)
    print(gain)
    return gain

# if __name__ == "__main__":

#     #how to initialize the device
#     print("INITIALIZING AND GETTING BASIC DEVICE INFORMATION")
#     print()
#     print()

#     #Establish the name and serial number of the instrument. Ensure that the serial number matches yours.
#     SC1 = SignalCore_SC5318A("SC1", "10002EA6")
#     print("Device opened!")

#     #displays the device's info
#     get_idn()

#     print("Reading the current temperature of the device...")
#     device_temp()

#     get_rf_frequency()
#     get_if_frequency()
#     get_lo_frequency()

#     print("Displaying the current RF parameters...")
#     display_rf_parameters()

#     #get the gain conversion
#     calc_gain()

#     #reading from the eeprom
#     read_eeprom(0, 0x04, 4)

#     #example of the reg_read and reg_write commands
#     #reads back the device's serial number
#     reg_read(0x30, 0)
#     #sets RF in freq to 1200000000000 Hz
#     reg_write(0x10, 1200000000000)

#     print("Changing the IF frequency value...")
#     set_if_frequency(1.2e9)
#     get_if_atten_value()

#     print("Closing device")
#     close_device()