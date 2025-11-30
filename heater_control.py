#!/usr/bin/env python3
import minimalmodbus
import time, json, signal, os, logging, traceback, sys
import paho.mqtt.client as mqtt
from dataclasses import dataclass
from dataclasses_json import dataclass_json
from threading import Event
from typing import *

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
class Temp:
    set_point: Temp_Item
    filter_ratio: Temp_Item
    hysteresis: Temp_Item

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
        temp: Temp
        mqtt: MQTT
        modbus: Modbus

    exit = Event()

    CHAR_LUT = [
        0x3f, 0x06, 0x5b, 0x4f,
        0x66, 0x6d, 0x7d, 0x07,
        0x7f, 0x6f, 0x77, 0x7c,
        0x39, 0x5e, 0x79, 0x71
    ]

    DIGITS = 4

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
        half_hysteresis = self.config.temp.hysteresis.val // 2
        """Determine heater on/off command"""
        self.low_point = self.config.temp.set_point.val - half_hysteresis
        self.high_point = self.config.temp.set_point.val + half_hysteresis 
        if self.fil.y > (self.high_point / 10.0):
            self.heater_on = 0
        elif (self.meas_temp // 100) < self.low_point:
            self.heater_on = 1
        """Build our response message"""
        temp_dict = {"HeaterControl Fil Temp" : int(round(self.fil.y * 1000.0)),
                     "HeaterControl Set High" : self.high_point * 100,
                     "HeaterControl Set Low" : self.low_point * 100,
                     "HeaterControl On" : self.heater_on,
                     "time": time.time()}
        logging.info(f"Publishing: {str(temp_dict)}")
        self.publish(f"{self.config.name}/event", json.dumps(temp_dict))

    def signal_handler(self, signum, _):
        """signal handling helper function"""
        logging.warning(f"Caught a deadly signal: {signum}")
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

    def main(self):
        """this is the main function and most of the work in this script"""
        nextWait = 0.250;
        start = time.monotonic()
        last_cycle_overrun = False
        self.main_cycle = 0

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.instr = minimalmodbus.Instrument(self.config.modbus.port, self.config.modbus.sid)
        self.instr.serial.baudrate = self.config.modbus.baud
        self.instr.serial.timeout = self.config.modbus.timeout
        self.instr.serial.clear_buffers_before_each_transaction = False

        self.fil = HC.Temp_IIR(self.config.temp.set_point.val / 10.0, self.config.temp.filter_ratio.val / 1000.0)
        self.meas_temp = self.config.temp.set_point.val * 100
        self.heater_on = 0

        button = 0
        last_button = 0
        disp = []
        last_disp = []

        self.connect(host=self.config.mqtt.broker, port=self.config.mqtt.port,
                     keepalive=self.config.mqtt.timeout)
        self.loop_start()

        start = time.monotonic()
        while not self.exit.wait(nextWait):
            if (self.main_cycle >= 65535):
                self.main_cycle = 0
            else:
                self.main_cycle += 1
            """get button press here"""
            last_button = button
            button = self.handle_modbus(self.instr.read_register, 0, functioncode = 4)
            """ TODO: break out into number editor """
            if (last_button != button) and (button & self.KP_ACT):
                button_code = button & (self.KP_ACT - 1)
                logging.debug(f"Button Press Recorded: {button_code}")
                if button_code == self.KP_UP:
                    self.config.temp.set_point.val += 1
                    logging.info(f"Temp inc to {self.config.temp.set_point.val / 10.0}")
                    self.process_temp()
                elif button_code == self.KP_DWN:
                    self.config.temp.set_point.val -= 1
                    logging.info(f"Temp dec to {self.config.temp.set_point.val/ 10.0}")
                    self.process_temp()
                
            """process on off here"""
            self.handle_modbus(self.instr.write_bit, 0, self.heater_on)
            """output display here"""
            sp_str = str(self.config.temp.set_point.val)
            last_disp = disp
            disp = self.str27seg(sp_str, 1)
            even_odd = self.main_cycle % 2
            self.handle_modbus(self.instr.write_register, even_odd, disp[even_odd])

            """ Main Loop Execution Rate Handling """
            curTime = time.monotonic()
            nextWait = 0.100
            nextWait -= curTime - start
            if nextWait < 0.0:
                if last_cycle_overrun == 0:
                    logging.debug("Main loop overrun")
                elif last_cycle_overrun >= 20:
                    last_cycle_overrun = 0
                start = curTime + nextWait
                nextWait = 0.0
                last_cycle_overrun += 1
            else: 
                start = curTime
                last_cycle_overrun = 0

        self.disconnect()
        self.loop_stop()

if __name__ == "__main__":
    heaterControl = HC(mqtt.CallbackAPIVersion.VERSION2, "heater_control")
    my_path = os.path.dirname(os.path.abspath(__file__))
    logging.basicConfig(level=logging.INFO)
    logging.info("Reading Config File")
    with open(f"{my_path}{os.sep}hc_config.json", "r") as configFile:
        heaterControl.config = HC.Config.from_json(configFile.read())
        logging.debug(f"{heaterControl.config}")
    logging.info("Starting")
    heaterControl.main()

