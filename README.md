# coSimple - Simple CANopen master functionality.

Minimalistic and simple to use CANopen master. Implements no object directory and is for sure not CiA301 compliant. But it allows the control of CANopen nodes on extreme constrained and bare metal environments. For more fully fledged masters with automatic configuration based on dcf files and other goodies see other libraries like [LeLy-core](https://opensource.lely.com/canopen/) from LeLy industries.


## Why?

I needed a simple and low memory footprint CANopen master for an embedded project. Other implementations tried to do too much, were hard to use or just had no available C API. Given a slave that implements the CiA301 _Default Connection Set_, a master really does not have to be all that complicated. Indeed `coSimple` is merely a thin wrapper around bare CAN rx and tx functions and exposes a simple API to the CANopen NMT, SYNC, EMCY, PDO and SDO services.


## What?

This library implements:
 - NMT master
 - SYNC producer
 - EMCY receiver
 - TIME producer
 - PDO receive/transmit
     - only one PDO for each
 - SDO client
     - only expedited
     - only on default channels
     - only at max 4 byte data types, (u)int8 - (u)int32


## How?

Mode of operation:
 - reset/reboot node with NMT
 - configure node with SDO service
 - bring node in operational state with NMT
 - cyclically:
     - send PDO to node,         see `coTPDO()`
     - issue SYNC,               see `coSYNC()`
     - receive PDO from node,    see `coRPDO()`
     - received EMCY messages in this cyclic mode are forwarded to application
     - no SDO transactions supported in cyclic operation! (need to stop, reconfigure and start again)


## Links

- CANopen Explained - A Simple Intro: https://www.csselectronics.com/pages/canopen-tutorial-simple-intro
- Open Vehicles - CANopen Basics: https://docs.openvehicles.com/en/latest/components/canopen/docs/Intro.html


## License
[MIT](LICENSE) © N. Leuenberger.
