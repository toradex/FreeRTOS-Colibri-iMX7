# "flexcan_network" example on Colibri Evaluation Board

This document describes how to setup the Colibri Evaluation Board V3.2a (EvB)  in order to run the *flexcan_network*  example application. 

The relevant external connections are:

* X27: 
  UARTA. Debug messages for U-Boot and Linux

* X25 (upper)
  UARTB. Debug messages of the M4 application
* X2 (upper)
  CAN bus

Some internal connections on the EvB need to be reconfigured in order to be able to use the FlexCan:

* JP4, JP5, X9-5, X9-6: 
  Use FlexCan pins, instead of the MCP2515-Can-Controller
* JP17, JP19, JP20, JP21
  Use the on-board USB-to-UART converte for UARTA. 

The following sections describe the details, how to reconfigure the EvB:

## CAN pins

Background information: The FlexCan pins used in the example are:

| SODIMM Pin | Jumper | Function |
| ---------- | ------ | -------- |
| 55         | X8-5   | CAN1_TX  |
| 63         | X8-6   | CAN1_RX  |

The reconfigurations you need to do are:

| Part           | Connection     | Description                                         |
| -------------- | -------------- | --------------------------------------------------- |
| JP4<br />JP5   | remove jumpers | Disconnect CAN transceivers from MCP2515 controller |
| X8-5<br />X8-6 | remove jumpers | Disconnect SODIMM 55 and 63 from any circuits.      |
| X9-5           | to X38-1       | Connect FlexCAN CAN1_TX to CAN transceiver          |
| X9-6           | to X38-2       | Connect FlexCAN CAN1_RX to CAN transceiver          |

## UARTs

To route the UARTA through the on-board USB-to-UART converter:

| Part                                | Connection | Description                                             |
| ----------------------------------- | ---------- | ------------------------------------------------------- |
| JP17 <br />JP19<br />JP20<br />JP21 | Pos 2-3    | Route UARTA to the USB-to-UART converter (USB port X27) |



# External Connections

The following components are required outside of the EvB:

1. A connection to your PC for UARTA
2. A connection to your PC for UARTB
3. A CAN analyzer or another active CAN device
   One CAN node alone cannot be used to test CAN communication, not even for transmission. 
   You need to connect a 2nd *active* CAN device, for example a CAN analyzer 
4. The CAN bus requires a proper termination: 120&Omega; between pins 2 and 7 of the D-Sub9.