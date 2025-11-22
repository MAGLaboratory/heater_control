#!/usr/bin/env python3
import minimalmodbus
import time, json, signal, os, logging, traceback
import paho.mqtt.client as mqtt
from dataclasses import dataclass
from dataclasses_json import dataclass_json
from threading import Event
from typing import *

class HC(mqtt.Client):
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

    @dataclass_json
    @dataclass
    class Config:
        name: str
        comment: str
        secret: str
        temp: HC.Temp
        mqtt: HC.MQTT
        modbus: HC.Modbus

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
                logging.info(f"Decoded Message: {data}")
                if self.config.mqtt.temp_source in data:
                    logging.info(f"Received temperature: {data[self.config.mqtt.temp_source]}")
                    temp = int(data[self.config.mqtt.temp_source]) / 1000.0
                    logging.info(f"Filtered temperature: {self.fil.filt(temp)}")
                    temp_dict = {"HeaterControl Fil Temp" : int(round(self.fil.y * 1000.0)),
                                 "time": time.time()}
                    logging.info(f"Publishing: {str(temp_dict)}")
                    self.publish(f"{self.config.name}/event", json.dumps(temp_dict))
                else: 
                    logging.warning(f"Message received without {self.config.mqtt.temp_source}")
            except Exception as e:
                tb = traceback.format_exc()
                logging.warning(f"Caught exception: {e}\nTraceback:\n{tb}")


    def main(self):
        """this is the main function and most of the work in this script"""
        nextWait = 1.0;
        start = time.monotonic()
        last_cycle_overrun = False
        self.fil = HC.Temp_IIR(self.config.temp.filter_ratio)
        self.connect(host=self.config.mqtt.broker, port=self.config.mqtt.port, keepalive=self.config.mqtt.timeout)
        self.loop_start()
        while not self.exit.wait(nextWait):

            """get display action here""" 
            """process on off here"""
            """output display here"""

            """ Main Loop Execution Rate Handling """
            curTime = time.monotonic()
            nextWait = curTime - start + 1.0
            if nextWait < 0.0:
                if last_cycle_overrun == False:
                    logging.info("Main loop overrun")
                start = curTime + nextWait
                nextWait = 0.0
                last_cycle_overrun = True
            else: 
                start = curTime
                last_cycle_overrun = False

        self.disconnect()
        self.loop_stop()

if __name__ == "__main__":
    heaterControl = HC(mqtt.CallbackAPIVersion.VERSION2, "heater_control")
    my_path = os.path.dirname(os.path.abspath(__file__))
    logging.basicConfig(level=logging.INFO)
    logging.info("Opening Config File")
    with open(f"{my_path}{os.sep}hc_config.json", "r") as configFile:
        heaterControl.config = HC.Config.from_json(configFile.read())
    logging.info("Starting main function")
    heaterControl.main()

