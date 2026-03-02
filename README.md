# heater_control

The heater controls for the MAG Laboratory natural gas heater

## Branch Description
This is the "zero" branch where the initial development is supposed to take place.

## Future Work
This program may need to be converted to use the TCP version of modbus since
there are supposed to be multiple devices on this bus.

Additionally, the (kick) start timer may be improved if there is a linear
rise in the low-limit so that the script can more precisely track the set
point temperature.

## Description
This software consists of a few components which enable a user to interact with
a simple HMI which can set parameters and contains an internal relay to
activate the heater.

One can simplify this to a set of components:
* MQTT sensor input
* occupancy calculation
* heater control algorithm
* user interface
* heater output

### User Interface
The user interface accepts input from the buttons on the HMI and allows access
to several parameters.  There is a separation between three categories of
parameter:
* Read-only
  * Temperature
* Numberic
  * Set-Point
  * Filter Ratio
  * Hysteresis
  * Start Time
  * Occupancy Cold Time
* Binary
  * Force occupancy 
* Save

### Heater Control Algorithm
The heater control algorithm is a bang-bang control algorithm developed with
empirical data and a filter.

The high-point is set at the set-point and the low-limit is set at the
set-point minus hysteresis unless the (kick) start timer is expired.

The heater is turned on at the low-limit which is based directly on the
measured temperature.

The heter is turned off at the high-limit which is based on the filtered
temperature.

#### The Filter
The filter uses the classic floating point infinite impulse response (IIR)
filter.  The user can choose the filter ratio (or alpha) from the user
interface.

$$y[n] = (1-\alpha) y[n-1] + \alpha x[n]$$

#### (kick) Start Timer
The timer starts when the heater turrns off.  Upon expiration of the start
timer, the low-limit is set to half of the hysteresis value.

### Occupancy Determination
The occupancy of the makerspace is determined using a state machine.

This state machine has three states and begins with the `Cold` state:
* `Cold`
* `Warm`
* `Hot`

The state transitions from the `Cold` state to the `Hot` state if there is
more than one active motion sensor.

The state transitions from the `Hot` state to the `Warm` state automatically.

The state transitions from the `Warm` to the `Hot` state if there is at least
one active motion sensor.

The state transitions from the `Warm` to the `Cold` state if the state machine
has remained in the `Warm` state for too long.

The output of this state machine is binary:
* `True` if the state is *not* `Cold`
* `False` otherwise

## Dependencies
* pago-mqtt
* dataclasses-json
* minimalmodbus

## Installation
This script can be "installed" using the included systemd `.service` and
environment file.

The current configuration code allows for this script to be installed at
`/usr/local/bin` and its configuration to be stored in `/etc`.

## Help
Please do not hesitate to leave a Github issue or email the default MAG Laboratory contact [at] maglaboratory.org.

## Authors
* @blu006

## Version History
* TODO: first released version

## License
The Unlicense
