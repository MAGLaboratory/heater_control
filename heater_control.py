#!/usr/bin/env python3
import minimalmodbus
import time, json, signal, os, logging, traceback, sys, copy
import paho.mqtt.client as mqtt
from dataclasses import dataclass
from dataclasses_json import dataclass_json
from threading import Event
from typing import *
from ctypes import c_uint16

@dataclass_json
@dataclass
class Temp_Item:
    name7seg: Tuple[int, int]
    val: int
    decimal: int
    hi_lim: int
    lo_lim: int
    change_by: int = 1

@dataclass_json
@dataclass
class MQTT:
    data_sources: List[str]
    temp_source: str
    broker: str
    port: int
    timeout: int

@dataclass_json
@dataclass
class Modbus:
    ena: bool
    port: str
    sid: int
    timeout: float
    baud: int

class HC(mqtt.Client):
    @dataclass_json
    @dataclass
    class Config:
        name: str
        comment: str
        secret: str
        report_overrun: bool
        temp: Dict[str, Temp_Item]
        """
            must contain: 
            - set_point
            - filter_ratio
            - hysteresis
            - start_time
        """
        mqtt: MQTT
        modbus: Modbus
        loglevel: Optional[str] = None

    exit = Event()

    CHAR_LUT = [
        0x3f, 0x06, 0x5b, 0x4f,
        0x66, 0x6d, 0x7d, 0x07,
        0x7f, 0x6f, 0x77, 0x7c,
        0x39, 0x5e, 0x79, 0x71
    ]

    DIGITS = 4
    DS_TEMP = (0x7873, 0x0000)
    DS_OCC =  (0x3f58, 0x5800)
    DS_SAVE = (0x6d77, 0x1c79)
    DS_YES =  (0x6e79, 0x6d00)
    DS_NO =   (0x543f, 0x0000)

    KP_ACT = 0x40
    KP_UP  = 0x1C
    KP_LFT = 0x36
    KP_ENT = 0x24
    KP_RHT = 0X16
    KP_DWN = 0x26
    KP_BAK = 0x1E

    class Temp_IIR:
        first_sample_done = False
        y = 0.0
        a = 0.0
        def __init__(self, initial, alpha):
            self.first_sample_done = False
            self.y = initial
            self.a = alpha

        def filt(self, x):
            self.y = self.a*x + (1.0-self.a)*self.y if self.first_sample_done else x
            self.first_sample_done = True
            return self.y

    def process_temp(self):
        hysteresis = self.config.temp["hysteresis"].val
        """Determine heater on/off command"""
        if (self.start_cycle == True):
            self.low_point = self.config.temp["set_point"].val - hysteresis // 2
        else:
            self.low_point = self.config.temp["set_point"].val - hysteresis
        self.high_point = self.config.temp["set_point"].val 
        """ The filtered value is expressed as whole degrees """
        """ The measured temperature is expressed as thousandths of a degree"""
        if self.fil.y > (self.high_point / 10.0):
            self.heater_on = 0
        elif (self.meas_temp // 100) < self.low_point:
            self.heater_on = 1
        """Build our response message"""
        temp_dict = {"HeaterControl Fil Temp" : int(round(self.fil.y * 1000.0)),
                     "HeaterControl Set High" : self.high_point * 100,
                     "HeaterControl Set Low" : self.low_point * 100,
                     "HeaterControl Start Cycle" : int(self.start_cycle),
                     "HeaterControl On" : self.heater_on,
                     "time": time.time()}
        logging.info(f"Publishing: {str(temp_dict)}")
        self.publish(f"{self.config.name}/event", json.dumps(temp_dict))

    def signal_handler(self, signum, _):
        """signal handling helper function"""
        logging.critical(f"Caught a deadly signal: {signal.Signals(signum).name}")
        self.exit.set()

    def on_log(self, client, userdata, level, buf):
        if level == mqtt.MQTT_LOG_DEBUG:
            logging.debug("PAHO MQTT DEBUG: " + buff)
        elif level == mqtt.MQTT_LOG_INFO:
            logging.info("PAHO MQTT INFO: " + buff)
        elif level == mqtt.MQTT_LOG_NOTICE:
            logging.info("PAHO MQTT NOTICE: " + buff)
        elif level == mqtt.MQTT_LOG_WARNING:
            logging.warning("PAHO MQTT WARN: " + buff)
        else:
            logging.error("PAHO MQTT ERROR: " + buff)

    def on_connect(self, client, userdata, flags, rc, properties):
        """subscribes to the relevant channels"""
        if rc.is_failure:
            logging.warning(f"Temporarily to connect: {rc}.")
        else: 
            logging.info(f"Connected: {str(rc)}")
            for src in self.config.mqtt.data_sources:
                self.subscribe(src)

    def on_message(self, client, userdata, message):
        if message.topic in self.config.mqtt.data_sources:
            try:
                decoded = message.payload.decode('utf-8')
                logging.info(f"Received Message: {decoded}")
                data = json.loads(decoded)
                if self.config.mqtt.temp_source in data:
                    logging.info(f"Received temperature: {data[self.config.mqtt.temp_source]}")
                    self.meas_temp = int(data[self.config.mqtt.temp_source]) 
                    self.fil.filt(self.meas_temp / 1000.0)
                    self.process_temp()
                else: 
                    logging.warning(f"Message received without {self.config.mqtt.temp_source}")
            except Exception as err:
                tb = traceback.format_exc()
                logging.warning(f"L {sys._getframe().f_back.f_lineno} Caught exception: {err}\nTraceback:\n{tb}")

    def handle_modbus(self, fn, *params, **kwparams):
        tries = 5
        retval = None
        while tries > 0:
            tries -= 1
            try:
                retval = fn(*params, **kwparams)
                break
            except Exception as err:
                tb = traceback.format_exc()
                logging.warning(f"L {sys._getframe().f_back.f_lineno} Caught exception: {err}\nTraceback:\n{tb}")
                self.instr.serial.flush()
                self.instr.serial.reset_input_buffer()
                if tries <= 0:
                    raise

        return retval


    def str27seg(self, s, dig):
        rv = [0, 0]
        s = s[::-1]
        i = self.DIGITS
        while i != 0:
            """ The rest of this code expects the index to be one less than how
            it began.  This index is used to calculate the return vector """
            i -= 1
            """ Calculate inverse index """
            inv = self.DIGITS-1 - i
            num = 0
            try:
                """ Use the inverse index to look up the digit at that pos """
                num = int(s[inv])
            except IndexError:
                """ Print digit anyway because it is after or at the
                decimal point """
                if inv <= dig:
                    pass
                else:
                    break
            """ Compute the 7 segment vector for the display...
            There are two digits contained in each rv.
            The digit is looked up using the LUT and decimaled if appropriate
            then it is shifted into place. """
            rv[i // 2] |= ((self.CHAR_LUT[num] | (0x80 if inv == dig else 0))
                           << (0 if i % 2 else 8))
        return rv

    def param_sel(self, c_param, c_val, keycode):
        """ make sure the keycode is current """
        if (self.last_button != keycode) and (keycode & self.KP_ACT):
            """ hacky way to use the KP_ACT as an 'and' mask """
            button_code = keycode & (self.KP_ACT - 1)
            logging.debug(f"Button Press Recorded: {button_code}")
            if button_code == self.KP_UP:
                self.config.temp["set_point"].val += 1
                logging.info(f"Temp inc to {self.config.temp['set_point'].val / 10.0}")
                self.start_cycle = True
                self.process_temp()
            elif button_code == self.KP_DWN:
                self.config.temp["set_point"].val -= 1
                logging.info(f"Temp dec to {self.config.temp['set_point'].val/ 10.0}")
                self.start_cycle = True
                self.process_temp()

        self.last_button = keycode
        return c_param, c_val

    """
        The parameter display function takes a parameter index and a boolean.
        The boolean value `i_val` when `false` displays the parameter name and
        when `true` displays the value.

        The parameters are as follows:
        0 - temperature
        1 - set point
        2 - filter ratio 
        3 - hysteresis
        4 - start time
        5 - occupancy
        6 - save
    """
    def param_scroll(self, auto = True, i_param = 1, i_val = True):
        rv = [0, 0]
        val = True
        param = 1
        p_n = [0, 0]
        p_v = [0, 0]
        DIG_PARAM = len(self.config.temp.keys()) + 1
        if auto:
            param = self.main_cycle.value >> 4
            val = bool(param & 0x1)
            param >>= 1
            param %= DIG_PARAM + 1 # the save is not a regularly displayed parameter
        else:
            param = i_param
            val = i_val
        if param < 1:
            """ temperature """
            p_n = list(self.DS_TEMP)
            p_v = (str(self.meas_temp // 10), 2)
        elif param < DIG_PARAM:
            postParam = param - 1
            configKey = list(self.config.temp.keys())[postParam]
            configItem = self.config.temp[configKey]
            """ set point """
            """ filter ratio """
            """ hysteresis """
            """ start time """
            p_n = list(configItem.name7seg)
            p_v = [str(configItem.val), configItem.decimal]
        else:
            postParam = param - DIG_PARAM # + 1 - 1
            match postParam:
                case 0:
                    """ occupancy """ 
                    p_n = list(self.DS_OCC)
                    """ occupancy not implement yet """
                    p_v = list(self.DS_YES)
                case 1:
                    """ save """
                    p_n = list(self.DS_SAVE)

        """ Display either the parameter name or the value """
        if val == False:
            rv = p_n
        else:
            if (param < DIG_PARAM):
                rv = self.str27seg(*p_v)
            else: 
                rv = p_v

        return rv;

    def main(self):
        """this is the main function and most of the work in this script"""
        nextWait = 0.250;
        start = time.monotonic()
        last_cycle_overrun = 0
        self.main_cycle = c_uint16(0)
        last_cycle = self.main_cycle

        try:
            if type(logging.getLevelName(self.config.loglevel.upper())) is int:
                logging.basicConfig(level=self.config.loglevel.upper())
            else:
                logging.warning("Log level not configured.  Defaulting to WARNING.")
        except (KeyError, AttributeError) as e:
            logging.warning("Log level not configured.  Defaulting to WARNING.  Caught: " + str(e))

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.instr = minimalmodbus.Instrument(self.config.modbus.port, self.config.modbus.sid)
        self.instr.serial.baudrate = self.config.modbus.baud
        self.instr.serial.timeout = self.config.modbus.timeout
        self.instr.serial.clear_buffers_before_each_transaction = False

        self.fil = HC.Temp_IIR(self.config.temp["set_point"].val / 10.0, self.config.temp["filter_ratio"].val / 1000.0)
        self.meas_temp = self.config.temp["set_point"].val * 100
        self.start_cycle = True
        self.last_start_cycle = True
        self.start_cycle_timer = copy.copy(self.main_cycle)
        self.heater_on = 0

        button = 0
        self.last_button = 0
        disp = []
        last_disp = []
        param = 0
        val = True

        self.connect(host=self.config.mqtt.broker, port=self.config.mqtt.port,
                     keepalive=self.config.mqtt.timeout)
        self.loop_start()

        start = time.monotonic()
        while not self.exit.wait(nextWait):
            self.main_cycle.value += 1
            """get button press here"""
            button = self.handle_modbus(self.instr.read_register, 0, functioncode = 4)
            """ TODO: break out into number editor """
            param, val = self.param_sel(param, val, button)
                
            """process on off and start timer here"""
            self.handle_modbus(self.instr.write_bit, 0, self.heater_on)
            if (self.heater_on != 0):
                if (self.last_start_cycle == True):
                    logging.debug("Control start cycle deactivated")
                self.last_start_cycle = self.start_cycle
                self.start_cycle = False
                self.start_cycle_timer = copy.copy(self.main_cycle)
            else:
                st_val = self.config.temp["start_time"].val * 600
                if c_uint16(self.main_cycle.value - self.start_cycle_timer.value).value > st_val:
                    if (self.last_start_cycle == False):
                        logging.debug("Control start cycle activated")
                    self.last_start_cycle = self.start_cycle
                    self.start_cycle = True
                    self.start_cycle_timer.value = c_uint16(self.main_cycle.value - st_val - 1).value
            """output display here"""
            last_disp = disp
            disp = self.param_scroll()
            even_odd = self.main_cycle.value % 2
            self.handle_modbus(self.instr.write_register, even_odd, disp[even_odd])

            """ Main Loop Execution Rate Handling """
            curTime = time.monotonic()
            nextWait = 0.100
            nextWait -= curTime - start
            if nextWait < 0.0:
                """ Main loop overrun is only reported once """
                if last_cycle_overrun == 0 and self.config.report_overrun:
                    logging.debug("Main loop overrun")
                if last_cycle_overrun >= 20:
                    last_cycle_overrun = 0
                start = curTime + nextWait
                nextWait = 0.0
                last_cycle_overrun += 1
            else: 
                start = curTime
                last_cycle_overrun = 0

        self.disconnect()
        self.loop_stop()

""" 
    The default function in this script reads the configuration file found at
    where the source script exists.
"""
if __name__ == "__main__":
    heaterControl = HC(mqtt.CallbackAPIVersion.VERSION2, "heater_control")
    my_path = os.path.dirname(os.path.abspath(__file__))
    logging.basicConfig(level=logging.DEBUG)
    logging.info("Reading Config File")
    with open(f"{my_path}{os.sep}hc_config.json", "r") as configFile:
        heaterControl.config = HC.Config.from_json(configFile.read())
        logging.debug(f"{heaterControl.config}")
    logging.info("Starting")
    heaterControl.main()

