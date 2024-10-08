#!/usr/bin/env python3
from threading import Thread, Lock
import itertools
import serial
import time
import yaml
import numpy as np
import math
import pynmea2
import requests
import socket
from os import system
from datetime import datetime, timezone
import logging
from classes.mlat_solver_noproj import Mlat


def setup_logger(name, log_file, level=logging.INFO):
    """To setup as many loggers as you want"""

    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')

    handler = logging.FileHandler(log_file)
    handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger


# file loggers
log_full = setup_logger('log_full', '/home/pi/nav/beacons_full.log', level=logging.DEBUG)
log_short = setup_logger('log_short', '/home/pi/nav/beacons_short.log', level=logging.DEBUG)
log_broadcast = setup_logger('log_broadcast', '/home/pi/nav/data/broadcast.txt', level=logging.INFO)
log_unicast = setup_logger('log_unicast', '/home/pi/nav/data/unicast.txt', level=logging.INFO)
log_range = setup_logger('log_range', '/home/pi/nav/data/range.txt', level=logging.INFO)

# Parse config file
config_file = "/home/pi/nav/config.yaml"
#config_file = "/Users/jasmine/main/jnash/code/acoustic-beacons/config.yaml"
config = yaml.safe_load(open(config_file))

# Get general settings from the config file
settings = {
    'coords': config['settings']['coords'],
    'medium': config['settings']['medium'],
    'atm_pressure_hpa': float(config['settings']['atm_pressure_hpa']),
    'repeat_rate': float(config['settings']['repeat_rate']),
    'range_rate': float(config['settings']['range_rate']),
    'reply_timeout': float(config['settings']['reply_timeout']),
    'pressure_rate': float(config['settings']['pressure_rate']),
    'gps_fwd_rate': float(config['settings']['gps_fwd_rate']),
    'max_range_lag': float(config['settings']['max_range_lag']),
    'max_gps_lag': float(config['settings']['max_gps_lag']),
    'output_port': float(config['settings']['output_port']),
    'input_port': float(config['settings']['input_port']),
    'serial_beacon': config['settings']['serial_beacon'],
}


class Modem:
    """A class for communications with Delphis Subsea Modems"""

    # ======================================================
    # Process config file
    # ======================================================
    def __init__(self, mode=None, args=None):



        self.clientIP = None

        # self.lock = Lock()

        self.port = settings['serial_beacon']
        
        # sleep for 6 seconds to give the beacon time to load
        time.sleep(6)

        # Open serial connection to modem
        self.ser = serial.Serial(
            port=self.port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        # Verify modem status
        status_msg = self.status()
        self.address = status_msg['src']
        log_full.info("Connected to modem %d, voltage: %.2fV" % (self.address, status_msg['voltage']))
        log_short.info("Connected to modem %d, voltage: %.2fV" % (self.address, status_msg['voltage']))
        print("Connected to modem %d, voltage: %.2fV" % (self.address, status_msg['voltage']))

        # Fall back to mode in config file if mode not set
        self.mode = mode or config['modems'][self.address]['mode']
        log_full.info("Starting in %s mode" % self.mode)
        log_short.info("Starting in %s mode" % self.mode)
        print("Starting in %s mode" % self.mode)
        self.args = args

        # Open GPS serial port if configured
        self.has_gps = False
        if 'serial_gps' in config['modems'][self.address]:
            log_full.debug("Opening GPS serial port")
            self.ser_gps = serial.Serial(
                port=config['modems'][self.address]['serial_gps'],
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            if hasattr(self, 'ser_gps'):
                if self.ser_gps.is_open:
                    self.has_gps = True

        # Open pressure serial port if configured
        # UNTESTED
        self.has_pressure = False
        if 'serial_pressure' in config['modems'][self.address]:
            log_full.debug("Opening pressure serial port")
            self.ser_pressure = serial.Serial(
                port=config['modems'][self.address]['serial_pressure'],
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            if hasattr(self, 'ser_pressure'):
                if self.ser_pressure.is_open:
                    self.has_pressure = True

        # Define all possible passive beacons
        self.passive_beacons = [int(m) for m in config['modems'].keys()
                                    if config['modems'][m]['mode'] == 'passive']

        # Initialize multilateration solver
        self.mlat = Mlat(config)

        # If stations were configured with local coordinates, convert them to a
        # local lat,lon coordinate system
        if config['settings']['coords'] == 'local':
            for m in self.passive_beacons:
                x = float(config['modems'][m]['x'])
                y = float(config['modems'][m]['y'])
                lat, lon = self.mlat.local2gps(x, y)
                config['modems'][m]['lat'] = lat
                config['modems'][m]['lon'] = lon

        # Initialize dictionaries for passive beacon locations & distances
        self.locs = {m: {'lat': float(config['modems'][m]['lat']),
                         'lon': float(config['modems'][m]['lon']),
                         'z':   float(config['modems'][m]['z']),
                         'time': None,
                         'recent': False}
                     for m in self.passive_beacons}
        self.dists = {m: {'range': None,
                          'time': None,
                          'recent': False,
                          'pings_sent': 0,
                          'pings_rcvd': 0}
                      for m in self.passive_beacons}

        # Initialize the dictionary for the surface vessel
        self.surface_vessel = {'lat': None,
                               'lon': None,
                               'heading': None,
                               'z': 0,
                               'time': None,
                               'heading_time': None,
                               'recent': False}

        # Set initial position of this beacon
        self.lat = None
        self.lon = None
        self.z = None
        self.time = None
        self.recent = False

        # get the type of platform this beacon is on
        self.platform = None
        if 'rov' in config['modems'][self.address]:
            if config['modems'][self.address]['rov']:
                self.platform = 'rov'
        if 'vessel' in config['modems'][self.address]:
            if config['modems'][self.address]['vessel']:
                self.platform = 'vessel'

        # Get broadcast times from config file
        self.broadcast_times = None
        if 'broadcast_times' in config['modems'][self.address]:
            self.broadcast_times = config['modems'][self.address]['broadcast_times']
        self.all_broadcast_times = []
        for m in self.passive_beacons:
            if 'broadcast_times' in config['modems'][m]:
                self.all_broadcast_times.extend(config['modems'][m]['broadcast_times'])
                next_second = [t + 1 for t in config['modems'][m]['broadcast_times']]
                self.all_broadcast_times.extend(next_second)
        log_full.debug(self.all_broadcast_times)

        # Get data transfer information from config file
        self.has_data = False
        self.data_files = None
        self.data_rates = None
        self.data_method = None
        self.data_target = None
        if 'data_files' in config['modems'][self.address]:
            if 'data_rates' in config['modems'][self.address]:
                self.has_data = True
                self.data_files = config['modems'][self.address]['data_files']
                self.data_rates = config['modems'][self.address]['data_rates']
                # unicast data need a target
                if 'data_target' in config['modems'][self.address]:
                    self.data_target = config['modems'][self.address]['data_target']
                    self.data_method = 'unicast'
                else:
                    self.data_method = 'broadcast'

        # Get location to store incoming beacon data
        self.recvd_data_path = None
        if 'recvd_data_path' in config['modems'][self.address]:
            self.recvd_data_path = config['modems'][self.address]['recvd_data_path']

        # the speed of sound (in m/s) depends on the medium
        # Note: speed of sound increases with increasing
        #       temperature, salinity, and pressure
        if 'sound_speed' in config['settings']:
            settings['sound_speed'] = config['settings']['sound_speed']
        elif settings['medium'] == 'air':
            settings['sound_speed'] = 343.0  # range is 319 to 343
        elif settings['medium'] == 'freshwater':
            settings['sound_speed'] = 1447.0  # range is 1403 to 1481
        elif settings['medium'] == 'saltwater':
            settings['sound_speed'] = 1500.0  # range is 1450 to 1570
        else:
            settings['sound_speed'] = 1500.0

        # initial conditions for system time set from GPS
        self.gps_start_time = None
        self.timeset = False

        self.input_port = None
        self.output_port = None
        if 'input_port' in config['settings']:
            self.input_port = config['settings']['input_port']
        if 'output_port' in config['settings']:
            self.output_port = config['settings']['output_port']

    # ======================================================
    # Low-level modem commands
    # ======================================================
    def send(self, cmd=None, wait=False, n=1, prefix=None):
        "Send a command, optionally wait for the Nth response"

        # If a command is specified and wait==False, send the command and
        # immediately return.
        if cmd and not wait:
            # with self.lock:
            self.ser.write(cmd.encode())
            return None

        # If wait==True, then:
        # - Send the command (if a command is given)
        # - Wait for the nth response with the desired prefix(es)
        #   - If no prefix(es) is specified,
        #     wait for nth response with any prefix.
        elif wait:
            t0 = time.time()
            c = 0
            while c < n and time.time() - t0 < settings['reply_timeout']:
                # Send command until at least 1 response is recieved,
                # then keep listening until nth response.
                if cmd and c == 0:
                    # with self.lock:
                    self.ser.write(cmd.encode())

                # If we get a response with the right prefix, increment the counter
                # with self.lock:
                response = self.ser.readline().decode().strip()
                if response:
                    if (not prefix):
                        c += 1
                    elif len(response) > 2:
                        if (response[1] in prefix):
                            c += 1
                time.sleep(settings['repeat_rate'])
            return parse_message(response)

    def status(self):
        "Query node address and voltage"
        cmd = "$?"
        return self.send(cmd=cmd, wait=True, prefix="A")

    def set_address(self, address):
        "Set node address"
        cmd = "$A%03d" % (address)
        return self.send(cmd=cmd, wait=True, prefix="A")

    def broadcast(self, message, wait=False):
        "Send message to all units in range"
        cmd = "$B%02d%s" % (len(message), message)
        log_full.debug(cmd)
        return self.send(cmd=cmd, wait=wait)

    def unicast(self, message, target, wait=False):
        "Send message to target unit (specified by 3-digit integer)"
        cmd = "$U%03d%02d%s" % (target, len(message), message)
        log_full.debug(cmd)
        return self.send(cmd=cmd, wait=wait)

    def ping(self, target, wait=False):
        "Send range ping to target unit"
        cmd = "$P%03d" % (target)
        return self.send(cmd=cmd, prefix=["P", "R"], n=2, wait=wait)


    def send_client(self, msg):
        """ Send a message to a client"""
        log_full.info('sending to client socket')
        
        if self.clientIP is not None:
            try:
                HOST = self.clientIP
                PORT = 4001
                s = socket.socket()
                s.connect((HOST, PORT))
                s.send(msg.encode())
                s.close()

            except Exception as e:
                print(e)
                print(HOST)
                print(port)



    # ======================================================
    # Processing threads
    # ======================================================
    # We should be fine to run any number of threads as long as:
    # - No two threads try to write to the same serial port at the same time
    # - No two threads try to read from the same serial port at the same time
    #
    # I've tried to set this up so any given thread only reads or only writes,
    # and only to one serial port.

    def active_ping(self):
        "Cyclically loop over passive beacons and send ranging pings"
        # Writes to acoustic modem serial port
        log_full.debug('Starting active ping thread.')
        sleep_to_full_second()
        t0 = time.time() - settings['range_rate']
        now = datetime.now(timezone.utc)
        for target in itertools.cycle(self.passive_beacons):
            while (time.time() - t0 <= settings['range_rate']) or (now.second in self.all_broadcast_times):
                time.sleep(0.05)
                now = datetime.now(timezone.utc)
            self.ping(target, wait=False)
            log_full.debug('Sent active ping.')
            self.dists[target]['pings_sent'] = self.dists[target]['pings_sent'] + 1
            t0 = time.time()

    def active_listen(self):
        "Parse ranging returns and broadcasts, update positions & distances"
        log_full.debug('Starting active listen thread.')
        try:
            while self.ser.is_open:
                # with self.lock:
                msg_str = self.ser.readline().decode().strip()
                log_full.debug(msg_str)
                msg = parse_message(msg_str)

                # Update position or distance from passive beacon
                if msg:
                    try:
                        run_update = False
                        msgtime = msg['time'].strftime("%Y-%m-%d %H:%M:%S.%f %Z")
                        if msg['type'] == 'broadcast':
                            log_full.debug('Got broadcast message.')
                            if is_hex(msg['str']):
                                # Decode the lat/lon message
                                lat, lon, z = decode_ll(msg['str'])
                                # Update the location entry for the source beacon
                                self.locs[msg['src']]['time'] = msg['time']
                                self.locs[msg['src']]['lat'] = lat
                                self.locs[msg['src']]['lon'] = lon
                                self.locs[msg['src']]['z'] = z
                                data = "%.5fN,%.5fE,%.1fm" % (lat, lon, z)
                                log_full.info("Message time %s" % (msg['time'],))
                                log_full.info("%d is at: %s" % (msg['src'], data))
                                run_update = True
                            else:
                                data = msg['str']
                                log_full.info("Message time %s" % (msg['time'],))
                                log_full.info("%d sent data message: %s" % (msg['src'], data))
                            line = str(msg['src']) + ' --- ' + str(data)
                            log_broadcast.info(line)
                        elif msg['type'] == 'unicast':
                            log_full.debug('Got unicast message.')
                            log_full.info(msg)
                            data = msg['str']
                            log_full.info("Message time %s" % (msg['time'],))
                            log_full.info("Unicast message received: %s" % (data,))
                            line = str(data)
                            log_unicast.info(line)
                            self.send_client(data)
                            send_socket(self.output_port, data)
                        elif msg['type'] == 'range':
                            log_full.debug('Got range message.')
                            # Update the distance entry for the source beacon
                            self.dists[msg['src']]['time'] = msg['time']
                            self.dists[msg['src']]['range'] = msg['range']
                            self.dists[msg['src']]['pings_rcvd'] = self.dists[msg['src']]['pings_rcvd'] + 1
                            log_full.info("Message time %s" % (msg['time'],))
                            log_full.info("%.2f m from %d" % (msg['range'], msg['src']))
                            line = str(msg['src']) + ' -- ' + str(msg['range'])
                            log_range.info(line)
                            run_update = True

                        if run_update:
                            # check which beacons have recent distances
                            now = datetime.now(timezone.utc)
                            max_range_lag = settings['max_range_lag']
                            max_gps_lag = settings['max_gps_lag']
                            for m in self.dists.keys():
                                self.dists[m]['recent'] = False
                                if self.dists[m]['time'] is not None:
                                    if (now - self.dists[m]['time']).total_seconds() < max_range_lag:
                                        self.dists[m]['recent'] = True
                            for m in self.locs.keys():
                                self.locs[m]['recent'] = False
                                if 'static' in config['modems'][m]:
                                    if config['modems'][m]['static']:
                                        self.locs[m]['recent'] = True
                                    else:
                                        if self.locs[m]['time'] is not None:
                                            if (now - self.locs[m]['time']).total_seconds() < max_gps_lag:
                                                self.locs[m]['recent'] = True
                                else:
                                    if self.locs[m]['time'] is not None:
                                        if (now - self.locs[m]['time']).total_seconds() < max_gps_lag:
                                            self.locs[m]['recent'] = True
                            self.recent_beacons = []
                            for m in self.passive_beacons:
                                if self.dists[m]['recent'] and self.locs[m]['recent']:
                                    self.recent_beacons.append(m)
                            log_full.debug('Recent beacons: ')
                            log_full.debug(self.recent_beacons)

                            # Pass positions & distances to multilateration solver
                            if len(self.recent_beacons) > 1:

                                # Use previous position as initial guess if possible
                                if not (self.lat and self.lon and self.z):
                                    x0 = None
                                else:
                                    x0 = np.array((self.lat, self.lon, self.z))

                                recent_locs = dict()
                                recent_dists = dict()
                                for m in self.recent_beacons:
                                    recent_locs[m] = self.locs[m]
                                    recent_dists[m] = self.dists[m]

                                log_full.debug('Starting solve ...')
                                [lat, lon, z] = self.mlat.solve(recent_locs, recent_dists, x0=x0)
                                log_full.debug('Finished solve.')

                                self.lat = lat
                                self.lon = lon

                                # keep the output depth only if we don't have pressure data
                                if not self.has_pressure:
                                    self.z = z
                                if not self.z:
                                    self.z = z

                                log_full.info("Location of this active beacon: %.6f, %.6f, %.2f" % (self.lat, self.lon, self.z))
                                log_short.info('This beacon: Calculated location: %.6f, %.6f, %.2f' % (self.lat, self.lon, self.z))

                            else:
                                log_full.info("Location of this active beacon not calculated.")
                                log_short.info('This beacon: Calculated location: Not calculated.')

                            # write status to short log
                            for m in self.passive_beacons:
                                log_short.info('Beacon %d: Recent range   : %s' % (m, self.dists[m]['recent']))
                                log_short.info('Beacon %d: Recent locat   : %s' % (m, self.locs[m]['recent']))
                                log_short.info('Beacon %d: Last range time: %s' % (m, self.dists[m]['time']))
                                log_short.info('Beacon %d: Last locat time: %s' % (m, self.locs[m]['time']))
                                log_short.info('Beacon %d: Pings rcvd/sent: %d/%d' % (m, self.dists[m]['pings_rcvd'], self.dists[m]['pings_sent']))
                                log_short.info('Beacon %d: Last range     : %s' % (m, str(self.dists[m]['range'])))
                                log_short.info('Beacon %d: Last locat     : %s, %s, %s m' % (m, str(self.locs[m]['lat']), str(self.locs[m]['lon']), str(self.locs[m]['z'])))

                            log_full.handlers[0].flush()
                            log_short.handlers[0].flush()

                    except Exception as e:
                        log_full.debug(e)
                        log_full.debug('active_listen error')

            log_full.info('Beacon serial port closed.')

        except Exception as e:
            logging.debug(e)
 
        log_full.info('Active listening ended.')


    def passive_listen(self):
        "Parse and forward any broadcasts/unicasts received"
        while self.ser.is_open:
            # with self.lock:
            msg_str = self.ser.readline().decode().strip()
            log_full.debug(msg_str)
            msg = parse_message(msg_str)

            # Forward any commands received to a socket
            if msg:
                data = msg['str']
                log_full.debug("Received broadcast/unicast message.")
                log_full.info("Message time %s" % (msg['time'],))
                log_full.info("Received message: %s" % (data,))
                send_socket(self.output_port, data)

    def monitor_gps(self):
        """Parse all incoming GPS messages and update position
        """
        # Reads from GPS serial port
        while self.ser_gps.is_open:

            msg_str = self.ser_gps.readline().decode('utf-8', 'ignore').strip()
            if msg_str:
                # log_full.debug(msg_str)
                if len(msg_str) > 5:
                    if msg_str[0:6] == "$GPGGA":
                        log_full.debug(msg_str)
                        try:
                            parsed = pynmea2.parse(msg_str)
                            if parsed.latitude and parsed.longitude:
                                self.lat = parsed.latitude
                                self.lon = parsed.longitude

                        except Exception as e:
                            log_full.error('Unable to parse GPS GGA message.')
                            log_full.error(e)
                    elif not self.timeset and msg_str[0:6] == "$GPRMC":
                        log_full.debug(msg_str)
                        try:
                            rmc = pynmea2.parse(msg_str)
                            if (rmc.datestamp is not None) and (rmc.timestamp is not None):
                                self.gps_start_time = datetime.combine(
                                    rmc.datestamp, rmc.timestamp)
                                log_full.debug(self.gps_start_time)
                                if self.gps_start_time:
                                    set_system_time(self.gps_start_time)
                                    self.timeset = True
                        except Exception as e:
                            log_full.error('Unable to parse GPS RMC message.')
                            log_full.error(e)

    def monitor_vessel_gps(self):
        """Parse all incoming UDP vessel GPS messages
        and updates vessel beacon positions
        """

        log_full.debug('Starting monitor_vessel_gps thread.')

        # Reads from GPS UDP port
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:

            s.bind(('192.168.2.2', 14401))
            log_full.debug('Bound to socket %s' % (s))
            while True:
                data, addr = s.recvfrom(1024)  # buffer size is 1024 bytes

                msg_str = data.decode('utf-8').strip()
                log_full.debug('Vessel GPS: %s' % (msg_str))
                if msg_str:
                    if len(msg_str) > 5:

                        if msg_str[0:6] == "$GPGGA":
                            log_full.debug('Vessel GGA: %s' % (msg_str))
                            try:
                                parsed = pynmea2.parse(msg_str)
                                if parsed.latitude and parsed.longitude:
                                    self.surface_vessel['lat'] = parsed.latitude
                                    self.surface_vessel['lon'] = parsed.longitude
                                    self.surface_vessel['time'] = datetime.now(timezone.utc)
                                else:
                                    self.surface_vessel['lat'] = None
                                    self.surface_vessel['lon'] = None
                                    self.surface_vessel['time'] = None
                            except Exception as e:
                                log_full.error('Unable to parse vessel GPS GGA message.')
                                log_full.error(e)

                        if msg_str[0:6] == "$HEHDT":
                            log_full.debug('Vessel HDT: %s' % (msg_str))
                            try:
                                parsed = pynmea2.parse(msg_str)
                                if parsed.heading:
                                    self.surface_vessel['heading'] = parsed.heading
                                    self.surface_vessel['heading_time'] = datetime.now(timezone.utc)
                                else:
                                    self.surface_vessel['heading'] = None
                                    self.surface_vessel['heading_time'] = None
                            except Exception as e:
                                log_full.error('Unable to parse vessel GPS HDT message.')
                                log_full.error(e)

                        # for every GGA message received,
                        # only calculate vessel beacon positions if
                        # the vessel heading and latlon messages were received
                        # within 2 seconds of each other
                        if msg_str[0:6] == "$GPGGA":
                            if (self.surface_vessel['time'] is not None) and (self.surface_vessel['heading_time'] is not None):
                                timediff = (self.surface_vessel['heading_time'] - self.surface_vessel['time']).total_seconds()
                                if abs(timediff) <= 2:
                                    self.locate_vessel_beacons()

        log_full.debug('Ending monitor_vessel_gps thread.')

    def locate_vessel_beacons(self):
        """ Calculates the realtime position, in lat/lon coordinates, of each of
        the beacons attached to a surface vessel.  Uses the vessel GPS
        lat/lon/heading, and the static offsets of each of the beacons. """

        # surface vessel information
        vlat = self.surface_vessel['lat']
        vlon = self.surface_vessel['lon']
        vhead = self.surface_vessel['heading']
        vtime = self.surface_vessel['time']

        for m in self.passive_beacons:
            if 'vessel' in config['modems'][m]:
                if config['modems'][m]['vessel']:
                    x = float(config['modems'][m]['x'])
                    y = float(config['modems'][m]['y'])
                    lat, lon = gps_at_offset(vlat, vlon, vhead, x, y)
                    self.locs[m]['lat'] = lat
                    self.locs[m]['lon'] = lon
                    self.locs[m]['time'] = vtime
                    log_full.debug('Modem %s at %s' % (m, self.locs[m]))

    def monitor_rov_pressure(self):
        """ Request ROV pressure (hPa) from PixHawk
        """
        period = settings['pressure_rate']
        api_url = "http://192.168.2.2:4777/mavlink/SCALED_PRESSURE2/press_abs"
        # give the pressure sensor time to warm up
        time.sleep(20)
        while True:
            try:
                response = requests.get(api_url, timeout=0.5)
                output = response.json()
                self.pressure = output
                now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f %Z")
                self.z = pressure_2_depth(self.pressure, settings['atm_pressure_hpa'])
                log_full.debug('Pressure: ' + str(self.pressure) + ' hPa')
                log_full.debug('Depth: ' + str(self.z) + ' m')
                pline = now + ' -- ' + str(self.pressure) + ' hPa'
                dline = now + ' -- ' + str(self.z) + ' m'
                write_to_file(self.recvd_data_path + '/rov_pressure.txt', pline)
                write_to_file(self.recvd_data_path + '/rov_depth.txt', dline)
                self.has_pressure = True
            except Exception as e:
                log_full.error('Unable to parse pressure from API.')
            time.sleep(period)

    def monitor_ser_pressure(self):
        """ Collect pressure data from serial data feed
        UNTESTED, TODO
        """
        while self.ser_pressure.is_open:
            msg_str = self.ser_pressure.readline().decode().strip()
            if msg_str:
                log_full.debug(msg_str)
                # TODO: parse pressure messages and assign pressure
                # self.pressure = ...
                # self.z = pressure_2_depth(self.pressure, settings['atm_pressure_hpa'])

    def monitor_pressure(self):
        """ Wrapper for the various pressure data sources."""
        if hasattr(self, 'ser_pressure'):
            self.monitor_ser_pressure()
        elif 'rov' in config['modems'][self.address]:
            if config['modems'][self.address]['rov']:
                self.monitor_rov_pressure()
        else:
            self.z = config['modems'][self.address]['z']
            log_full.warning('No pressure available. Using config depth.')

    def passive_broadcast(self):
        """Periodically broadcast current gps position and depth.
        TO DO: add a sleep calculation so we don't miss any broadcasts."""
        sleep_to_full_second()
        if self.has_gps or self.has_pressure:
            while self.ser.is_open:
                now = datetime.now(timezone.utc)
                if now.second in self.broadcast_times:
                    if self.has_gps and self.lat and self.lon:
                        log_full.info("Passive beacon position is %.5f, %.5f at %.2f m" % (self.lat, self.lon, self.z))
                        log_short.info("Passive beacon position is %.5f, %.5f at %.2f m" % (self.lat, self.lon, self.z))
                    elif self.has_pressure and self.z:
                        log_full.info("Passive beacon depth is %.2f m" % (self.z,))
                        log_short.info("Passive beacon depth is %.2f m" % (self.z,))
                    if (self.has_gps and self.lat and self.lon) or (self.has_pressure and self.z):
                        log_full.debug("Sending broadcast message.")
                        msg = encode_ll(self.lat, self.lon, self.z)
                        self.broadcast(msg)
                time.sleep(1)

    def passive_datacast(self, findex):
        """Periodically broadcast/unicast most recent data."""
        sleep_to_full_second()
        with open(self.data_files[findex], "r", encoding="utf-8") as f:
            while self.ser.is_open:
                # read the last line from the file
                for line in f:
                    pass
                msg = line
                # deliver the data
                if msg:
                    if self.data_method == 'broadcast':
                        log_full.debug("Sending broadcast data message.")
                        self.broadcast(msg)
                    elif self.data_method == 'unicast':
                        log_full.debug("Sending unicast data message.")
                        self.unicast(msg, self.data_target)
                time.sleep(self.data_rates[findex])

    def monitor_for_input(self):
        """ Monitor a socket for commands received external to this program.
        """

        log_full.debug('Starting monitor_for_input thread.')

        PORT = self.input_port
        HOST = '192.168.2.2'
        MAX_LENGTH = 4096

        log_full.debug('Opening input socket.')
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serversocket.bind((HOST, PORT))
        log_full.debug('Bound to socket %s' % (serversocket))
        serversocket.listen(5)



        while True:
            # accept connections from outside
            (clientsocket, address) = serversocket.accept()

            msg = clientsocket.recv(MAX_LENGTH)

            clientIP, clientPort = address            
            self.clientIP = clientIP

            log_full.debug('Client socket connected: ' + clientIP)
           
            self.send_client("ping")




            if msg == '':  # client terminated connection
                clientsocket.close()
                log_full.debug('Client socket closed.')
            strmsg = msg.decode()
            log_full.info(strmsg)
            log_short.info(strmsg)
            self.manage_input(strmsg)

        log_full.debug('Ending monitor_for_input thread.')

    def manage_input(self, msg):
        """ Unicast received commands to the desired target."""

        log_full.debug('Managing input.')
        log_full.debug(msg)


        try: 
            split_msg = msg.split(' ', 1)
            target = int(split_msg[0])
            cmd = split_msg[1]
            log_full.debug('Trying to unicast.')
            self.unicast(cmd, target)
            log_full.debug('Done managing input.')
        except:
            self.send_client("Bad input")

    def gps_forward(self):
        """Periodically forward current position to PixHawk
        """
        # NOTE: The only GGA fields parsed by the code that consumes
        # these messages on the ROV (nmea-receiver.py) are:
        # lat, lon, altitude, hdop, and satellites_visible.
        # Satellites_visible has to be at least 6 for the Pixhawk to
        # use the lat/lon data, so we hardcode it here as 6.
        sleep_to_full_second()
        if 'rov' in config['modems'][self.address]:
            if config['modems'][self.address]['rov']:

                fix_quality = '7'  # 7: manual input mode
                sat_vis = '07'  # has to be greater than 6 for Pixhawk to use it
                hdop = '1.0'  # ideal (<1), excellent (1-2), good (2-5)

                # GPS data are expected on port 27000
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    log_full.debug('Connecting to GPS UDP port.')

                    while True:
                        if self.lat is not None and self.lon is not None:
                            nowtime = datetime.utcnow().strftime('%H%M%S.00')
                            # convert lat/lon to NMEA format
                            [lat_nmea, lat_dir, lon_nmea, lon_dir] = ll_2_nmea(self.lat, self.lon)
                            # create a GGA NMEA sentence
                            msg_obj = pynmea2.GGA(
                                'GP', 'GGA',
                                (nowtime,
                                 lat_nmea, lat_dir,
                                 lon_nmea, lon_dir,
                                 fix_quality, sat_vis, hdop,
                                 str(-self.z), 'M', '', 'M', '', ''))
                            msg = str(msg_obj)
                            log_full.debug(msg)
                            msg_bytes = bytes(str(msg) + '\r\n', "ascii")
                            log_full.debug(msg_bytes)
                            # send gps position to ROV
                            s.sendto(msg_bytes, ('192.168.2.2', 27000))
                            log_full.debug('Forwarded GPS message to ROV.')
                        time.sleep(settings['gps_fwd_rate'])

    def debug_report(self):
        "Parse all incoming modem messages"
        # Reads from acoustic modem serial port
        while self.ser.is_open:
            # with self.lock:
            msg_str = self.ser.readline().decode().strip()
            msg = parse_message(msg_str)
            if msg:
                log_full.debug(msg)

    def debug_timer(self):
        "Periodically broadcast the current time"
        sleep_to_full_second()
        period = float(self.args[0]) - settings['rate']
        target = len(self.args) > 1 and int(self.args[1]) or None
        while self.ser.is_open:
            current_time = datetime.now(timezone.utc).strftime("%H:%M:%S")
            if target:
                self.unicast(current_time, target)
            else:
                self.broadcast(current_time)
            time.sleep(period)

    # ======================================================
    # Main loop
    # ======================================================
    def run(self, mode):

        # Set address and exit if in "set" mode
        if mode == "set":
            address = int(self.args[0])
            self.set_address(address)

        # Define threads, but don't start any
        ping_thread = Thread(target=self.active_ping)
        listen_thread = Thread(target=self.active_listen)
        gps_thread = Thread(target=self.monitor_gps)
        vessel_gps_thread = Thread(target=self.monitor_vessel_gps)
        broadcast_thread = Thread(target=self.passive_broadcast)
        command_thread = Thread(target=self.passive_listen)
        pressure_thread = Thread(target=self.monitor_pressure)
        output_thread = Thread(target=self.gps_forward)
        input_thread = Thread(target=self.monitor_for_input)

        # Threads for debugging
        report_thread = Thread(target=self.debug_report)
        timer_thread = Thread(target=self.debug_timer)

        if mode == "active":
            ping_thread.start()
            listen_thread.start()
            pressure_thread.start()
            output_thread.start()
            input_thread.start()
            if self.platform == 'rov':
                vessel_gps_thread.start()

        elif mode == "passive":
            if self.has_data:
                datathreads = list()
                for i in range(0, len(self.data_files)):
                    datacast_thread = Thread(target=self.passive_datacast, args=(i,))
                    datathreads.append(datacast_thread)
                    datacast_thread.start()
            input_thread.start()
            command_thread.start()
            # broadcast_thread.start()
            # gps_thread.start()
            # pressure_thread.start()
            pass

        elif mode == "timer":
            timer_thread.start()
            report_thread.start()

        elif mode == "report":
            report_thread.start()

# =========================================================================
# Helper functions
# =========================================================================


def parse_message(msg_str):
    "Parse a raw message string and return a useful structure"
    if not msg_str:
        return None
    elif len(msg_str) < 2:
        return None
    else:
        log_full.debug(':::' + msg_str + ':::')

    # Get message prefix and initialize output
    prefix = msg_str[0:2]
    msg = {}

    # time message received (timezone aware)
    msg['time'] = datetime.now(timezone.utc)

    try:

        # Status: #AxxxVyyyyy in response to "$?" (query status)
        #      or #Axxx       in response to "$Axxx" (set address)
        if prefix == "#A":
            msg['type'] = "status"
            msg['src'] = int(msg_str[2:5])  # xxx
            if len(msg_str) >= 11:
                msg['voltage'] = float(msg_str[6:11])*15/65536  # yyyyy...
            else:
                msg['voltage'] = None

        # Broadcast: #Bxxxnnddd..Qzz.. (broadcast received)
        #            #Bnn          (self broadcast acknowledge)
        elif prefix == "#B":
            if len(msg_str) > 4:
                msg['type'] = "broadcast"
                msg['src'] = int(msg_str[2:5])  # xxx
                msg['len'] = int(msg_str[5:7])  # nn
                end_data = 7 + msg['len']
                msg['str'] = msg_str[7:end_data]  # ddd...
                # msg['qual'] = msg_str[end_data+1:end_data+3]  # zz
            else:
                msg['type'] = "broadcast_ack"
                msg['len'] = int(msg_str[2:])

        # Unicast: #Unnddd...Qzz
        elif prefix == "#U":
            msg['type'] = "unicast"
            msg['src'] = None
            msg['len'] = int(msg_str[2:4])  # nn
            end_data = 4 + msg['len']
            msg['str'] = msg_str[4:end_data]  # ddd...
            # msg['qual'] = msg_str[end_data+1:end_data+3]  # zz

        # Range: #RxxxTyyyyy
        elif prefix == "#R":
            msg['type'] = "range"
            msg['src'] = int(msg_str[2:5])
            msg['range'] = float(settings['sound_speed']) * 3.125e-5 * float(msg_str[6:11])

        # Note: Other message types are possible, but we don't currently use any of
        #       them. Return None if we encounter these.
        else:
            return None

    except Exception as e:
        return None

    # Return message structure
    return msg


def pressure_2_depth(P_hPa, atm_P_hPa):
    """ Converts pressure (hPa) to depth (m)
    using the hydrostatic water pressure formula.
    Removes atmospheric pressure first.
    UNTESTED
    """

    # remove atmospheric pressure
    P_corr = P_hPa - atm_P_hPa
    P = P_corr * 100   # convert hPa to Pa

    g = 9.80665  # m/s2, acceleration of gravity

    medium = config['settings']['medium']
    if medium == 'saltwater':
        rho = 1023.6  # kg/m3, saltwater density
    else:
        rho = 997.0474  # kg/m3, freshwater density

    z = P / (rho * g)  # depth in meters
    return z


def encode_decimal_deg(deg):
    "Encode decimal lat or lon to hexidecimal degrees, minutes, seconds"
    # Output string looks like:  [DD] [MM] [SSS]  [N]
    #                              |    |    |     |
    #                           Degrees | Seconds  |
    #                                 Minutes    1 if negative
    if deg:
        neg = deg < 0
        deg = abs(deg)
        mins = (deg-np.floor(deg))*60
        secs = (mins-np.floor(mins))*int('fff', 16)
        return "%02x%02x%03x%1x" % (int(np.floor(deg)),
                                    int(np.floor(mins)),
                                    int(np.floor(secs)),
                                    neg)
    else:
        # all zeros if degree is None
        return "%02x%02x%03x%1x" % (0, 0, 0, 0)


def encode_z(z):

    za = int(abs(z))
    return "%02x" % (za, )


def decode_hex_dms(dms):
    "Decode hexidecimal degrees,mins,secs to decimal degrees"
    if dms == '00000000':
        return None
    else:
        degs = int(dms[0:2], 16)
        mins = int(dms[2:4], 16)
        secs = int(dms[4:7], 16)*60/int('fff', 16)
        neg = bool(int(dms[7]))
        dec = degs + mins/60 + secs/60**2
        return neg and -1*dec or dec


def decode_hex_z(hexz):
    z = -1 * float(int(hexz, 16))
    return z


def encode_ll(lat, lon, z):
    return encode_decimal_deg(lat) + encode_decimal_deg(lon) + encode_z(z)


def decode_ll(hex_str):
    lat = decode_hex_dms(hex_str[0:8])
    lon = decode_hex_dms(hex_str[8:16])
    z = decode_hex_z(hex_str[16:18])
    return lat, lon, z


def ll_2_nmea(lat_deg, lon_deg):
    """ Converts decimal lat/lon degrees to NMEA format DDDMM.MMMMM.
    """

    lat_dir = 'N' if lat_deg >= 0 else 'S'
    lon_dir = 'E' if lon_deg >= 0 else 'W'

    lat_deg = abs(lat_deg)
    lat_mins = (lat_deg-np.floor(lat_deg))*60
    lat_nmea = "%02d%09.6f" % (int(np.floor(lat_deg)),
                               lat_mins)

    lon_deg = abs(lon_deg)
    lon_mins = (lon_deg-np.floor(lon_deg))*60
    lon_nmea = "%03d%09.6f" % (int(np.floor(lon_deg)),
                               lon_mins)

    return lat_nmea, lat_dir, lon_nmea, lon_dir


def is_hex(s):
    try:
        int(s, 16)
        return True
    except ValueError:
        return False


def set_system_time(dateandtime):
    """Set the system time to the input datestamp from the GPS."""

    log_full.info('Setting the system time ...')
    log_full.info('Current system time : ' + str(datetime.now(timezone.utc)))
    log_full.info('Current GPS time    : ' + str(dateandtime))

    # set the system time
    system('sudo date --set="%s"' % str(dateandtime))

    log_full.info('Current system time : ' + str(datetime.now(timezone.utc)))


def sleep_to_full_second():
    """Sleeps until the time is close to being on a full second."""

    ms = float(datetime.now(timezone.utc).microsecond)
    time.sleep((1000000.0 - ms)/1000000.0)


def gps_at_offset(lat, lon, head, x, y):
    """Calculates the lat/lon coordinates of an asset on a platform
    given its x/y offsets (in meters) from a known location.  Takes into account
    the heading of the platform."""

    earthRadius = 6371000  # meters

    # convert the known lat/lon and heading to radians
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    head_rad = math.radians(head)

    # calculate the offset direct distance in meters
    d = (x**2 + y**2)**(1/2)

    # calculate the offset angle in radians
    if (x == 0) & (y == 0):
        a = 0
    if (x >= 0) & (y > 0):
        a = math.atan(abs(x/y))
    if (x > 0) & (y <= 0):
        a = math.atan(abs(y/x)) + np.pi/2
    if (x <= 0) & (y < 0):
        a = math.atan(abs(x/y)) + np.pi
    if (x < 0) & (y >= 0):
        a = math.atan(abs(y/x)) + 3/2*np.pi

    # incorporate the platform heading
    bearing = a + head_rad
    if bearing > 2 * np.pi:
        bearing = bearing - 2 * np.pi

    # calculate the lat/lon of the offset asset
    lat2 = math.asin(math.sin(lat1)*math.cos(d/earthRadius) +
               math.cos(lat1)*math.sin(d/earthRadius)*math.cos(bearing))

    lon2 = lon1 + math.atan2(math.sin(bearing)*math.sin(d/earthRadius)*math.cos(lat1),
                 math.cos(d/earthRadius)-math.sin(lat1)*math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return lat2, lon2


def send_socket(port, msg):
    """ Send a message to a port (socket)."""
    log_full.info('sending to udp socket')
    try:
        HOST = '127.0.0.1'
        s = socket.socket()
        s.connect((HOST, int(port)))
        s.send(msg.encode())
        s.close()
    except Exception as e:
        print(e)
        print(HOST)
        print(port)


def write_to_file(file, msg):
    """ Writes (appends) message to a file."""

    with open(file, 'a') as f:
        f.write(msg.rstrip() + '\n')
