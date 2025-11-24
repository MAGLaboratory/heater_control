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
class Temp:
    set_point: float
    filter_ratio: float
    hysteresis: float

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

    class Temp_IIR:
        first_sample_done = False
        y = 0.0
        a = 0.0
        def __init__(self, alpha):
            self.first_sample_done = False
            self.a = alpha

        def filt(self, x):
            self.y = self.a*x + (1.0-self.a)*self.y if self.first_sample_done else x
            self.first_sample_done = True
            return self.y

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
                    half_hysteresis = self.config.temp.hysteresis / 2.0
                    logging.info(f"Received temperature: {data[self.config.mqtt.temp_source]}")
                    self.low_point = self.config.temp.set_point - half_hysteresis
                    self.high_point = self.config.temp.set_point + half_hysteresis 
                    meas_temp = int(data[self.config.mqtt.temp_source]) / 1000.0
                    self.fil.filt(meas_temp)
                    """Determine heater on/off command"""
                    if self.fil.y > self.high_point:
                        self.heater_on = 0
                    elif meas_temp < self.low_point:
                        self.heater_on = 1
                    """Build our response message"""
                    temp_dict = {"HeaterControl Fil Temp" : int(round(self.fil.y * 1000.0)),
                                 "HeaterControl Set High" : int(round(self.high_point * 1000.0)),
                                 "HeaterControl Set Low" : int(round(self.low_point * 1000.0)),
                                 "HeaterControl On" : self.heater_on,
                                 "time": time.time()}
                    logging.info(f"Publishing: {str(temp_dict)}")
                    self.publish(f"{self.config.name}/event", json.dumps(temp_dict))
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

    def main(self):
        """this is the main function and most of the work in this script"""
        nextWait = 0.250;
        start = time.monotonic()
        last_cycle_overrun = False

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.instr = minimalmodbus.Instrument(self.config.modbus.port, self.config.modbus.sid)
        self.instr.serial.baudrate = self.config.modbus.baud
        self.instr.serial.timeout = self.config.modbus.timeout
        self.instr.serial.clear_buffers_before_each_transaction = False

        self.fil = HC.Temp_IIR(self.config.temp.filter_ratio)
        self.heater_on = 0

        button = 0
        last_button = 0

        self.connect(host=self.config.mqtt.broker, port=self.config.mqtt.port,
                     keepalive=self.config.mqtt.timeout)
        self.loop_start()

        start = time.monotonic()
        while not self.exit.wait(nextWait):

            """get button press here"""
            last_button = button
            button = self.handle_modbus(self.instr.read_register, 0, functioncode = 4)
            if (last_button != button) and (button & 0x40):
                logging.info(f"Button Press Recorded: {button & (0x40 - 1)}")
            """process on off here"""
            self.handle_modbus(self.instr.write_bit, 0, self.heater_on)
            """output display here"""

            """ Main Loop Execution Rate Handling """
            curTime = time.monotonic()
            nextWait = 0.250
            nextWait -= curTime - start
            if nextWait < 0.0:
                if last_cycle_overrun == 0:
                    logging.info("Main loop overrun")
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
        logging.info(f"{heaterControl.config}")
    logging.info("Starting main function")
    heaterControl.main()

