/**
 * @file example.c
 * @author Niklaus Leuenberger <@NikLeberg>
 * @brief Demonstration example for minimalistic coSimple CANopen Master
 * @version 0.1
 * @date 2023-06-07
 *
 * @copyright Copyright (c) 2024 Niklaus Leuenberger
 *            SPDX-License-Identifier: MIT
 *
 * The following example tries to give a brief overwiew how coSimple is to be
 * used. It uses some pseudo code or not implemented functions to guide you how
 * to adapt this to your specific device and platform.
 * 
 * The application is achitected such that:
 *  - first the CAN bus is initialized
 *  - all slaves are reset via NMT request
 *  - it waits until the configured slave sends a boot-up message
 *  - read vendor information from slave via SDO service
 *  - configure slave PDO mapping via SDO service
 *  - the CAN RX interrupt is enabled
 *  - the slave is set to operational mode via NMT request
 *  - a cyclic timer interrupt of a fixed frequency is started
 * Now, with every timer interrupt, a SYNC is sent on the CANopen bus and the
 * slave responds with its current RxPDO. That input data is then processed, new
 * TxPDO data is calculated and subsequently sent. The loop then waits for the
 * next exchange.
 */


#include "main.h"
#include "coSimple.h"

#include <stdint.h>
#include <stdio.h>


/*
 * Function Declarations
 *
 */

static void canInit(void);
static int canRx(co_msg_t *msg);
static int canTx(const co_msg_t *msg);
static void coEMCY(uint8_t nodeId, uint16_t eec, uint8_t er, uint8_t *msef);
static uint32_t getMs(void);


/*
 * Type Definitions
 *
 */

/* ... */


/*
 * Variable Definitions
 *
 */

#define CAN_ID (127) //<! ID of the CANopen slave we communicate with
co_t co = {
    .rx = canRx,
    .tx = canTx,
    .emcy = coEMCY,
    .ms = getMs};    //<! coSimple instance
uint32_t errCnt;     //<! error counter

uint8_t tpdo[6] = {0}; //<! PDO process data to slave
uint8_t rpdo[8] = {0}; //<! PDO process data from slave
size_t len;            //<! length of PDO response, should be = 8

volatile bool rpdo_received = false;


/*
 * Function Definitions
 *
 */

int main() {
    int32_t ret = 0;

    // initialize subsystems
    canInit();

    // issue a node reset to all CAN nodes
    errCnt -= coNMTReq(&co, 0, CO_NMT_RST);

    // wait for boot up message from slave
    errCnt -= coNMTWaitBoot(&co, CAN_ID);

    // read vendor information from slave
    uint32_t data = 0;
    errCnt -= coSDOReadU32(&co, CAN_ID, 0x1018, 0x01, &data); // read vendor id (0x1018.1)
    printf("\nvendor-id: 0x%x", data);
    errCnt -= coSDOReadU32(&co, CAN_ID, 0x1018, 0x02, &data); // read product code (0x1018.2)
    printf("\nproduct code: 0x%x", data);
    errCnt -= coSDOReadU32(&co, CAN_ID, 0x1018, 0x03, &data); // read revision number (0x1018.3)
    printf("\nrevision number: 0x%x", data);
    errCnt -= coSDOReadU32(&co, CAN_ID, 0x1018, 0x04, &data); // read serial number (0x1018.4)
    printf("\nserial number: %u", data);

    // perform pdo mapping:
    // setup TPDO1 mapping, status + position + current
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1800, 0x01, 0xc00001ff); // invalidate TPDO1
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1a00, 0x00, 0x00000000); // reset PDO mapping
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1a00, 0x01, 0x60410010); // status word, uint16
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1a00, 0x02, 0x60640020); // position actual, int32
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1a00, 0x03, 0x60780010); // current actual, int16
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1a00, 0x00, 0x00000003); // three mapped objects
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1800, 0x02, 0x00000001); // set TPDO1 as synchronous on each SYNC
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1800, 0x01, 0x400001ff); // activate TPDO1
    // setup RPDO1 mapping, control + position
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1400, 0x01, 0xc000027f); // invalidate RPDO1
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1600, 0x00, 0x00000000); // reset PDO mapping
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1600, 0x01, 0x60400010); // control word, uint16
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1600, 0x02, 0x60c10120); // target position, int32
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1600, 0x00, 0x00000002); // two mapped objects
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1400, 0x02, 0x00000001); // set RPDO1 as synchronous on each SYNC
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x1400, 0x01, 0x4000027f); // activate RPDO1

    // set operation mode
    errCnt -= coSDOWriteU32(&co, CAN_ID, 0x6060, 0x00, 7); // modes of operation, 7 = interpolated position

    // additional custom settings
    // coSDOWriteU16(&co, CAN_ID, <object-id>, <sub-index>, <data>);
    // ...

    printf("\nConfiguration error count: %d\n", errCnt);

    // enable CAN interrupt
    // ... enable_irq

    // set slave to operational mode
    coNMTReq(&co, CAN_ID, CO_NMT_OP);

    // enable cyclic timer interrupt
    // ... enable_irq

    // process application states
    while (1) {
        if (rpdo_received) {
            rpdo_received = 0;

            // PDO has been received into rpdo array during CAN interrupt
            // service routine. We can proccess it here.
            uint16_t rx_status = *((uint16_t*)&rpdo[0]); // status word, u16
            int32_t rx_position = *((int32_t*)&rpdo[2]); // actual position, i32
            int16_t rx_current = *((int16_t*)&rpdo[6]);  // actual current, i16

            // process received inputs, calculate PID loops, etc.
            // ...

            // send calculated outputs as PDO to slave
            tpdo[0] = 0; // control word, u16
            tpdo[1] = 0;
            tpdo[2] = 0; // target position, i32
            tpdo[3] = 0;
            tpdo[4] = 0;
            tpdo[5] = 0;
            coTPDO(&co, CAN_ID, tpdo, 6);
        }
    }
}

static void canInit(void) {
    // This is device specific but generally the following shall be done:
    //  - reset CAN hardware
    //  - disable and clear all interrupts
    //  - open CAN acceptance filter for all frames
    //  - configure interrupt on CAN RX
    //  - enable CAN bus

    // It is important that CAN RX interrupts are only configured but not yet
    // enabled! Interrupts will be enabled once the system goes to cyclic
    // operation i.e. after SDO configuration has been done.

    return;
}

static int canRx(co_msg_t *msg) {
    // This is device specific but generally the fields cobId, len and the data
    // array have to be filled with the received CAN frame data. Supported are
    // only the standard 11-bit identifiers and only data frames.

    // The implementation shall be non-blocking. Otherwise coSimple is not able
    // to detect timeouts. If no CAN frame is ready to be received (receive
    // buffer is empy) then simply return 1 for "no new data". Otherwise fill
    // the co_msg_t msg fields and return 0 for "new data".
    return 1;
}

static int canTx(const co_msg_t *msg) {
    // This is again device specific. Take fields cobId, len and the data array
    // from the given co_msg_t and construct a device specific CAN frame.

    // This call can be built blocking or non-blocking. Implementation has to
    // ensure that an immediate second call to this function does handle the
    // case where the TX buffer may be full.

    // coSimple assumes, that every co_msg_t that has been sent via a call to
    // this canTx and did return status code 0 for "frame sent" has been
    // actually sent. If that is not the case bad things can happen. From minor
    // timeouts up to complete system deadlock (because a SYNC frame was not
    // sent and we indefinitely wait for the RxPDO which will never be sent).

    // You may return -1 as error status code so that coSimple can retry or
    // notify the higher level API call.
    return -1; // error
}

static void coEMCY(uint8_t nodeId, uint16_t eec, uint8_t er, uint8_t *msef) {
    // If coSimple receives EMCY frames, they simply get forwarded here. It does
    // not react in any other way. If the EMCY requires attention and e.g. the
    // CAN bus must be restarted, the slave must be rebooted etc. then thats up
    // for the implementation to do.
    printf("\nCANopen EMCY! nodeId: 0x%03x eec: 0x%04x er: 0x%02x msef:", nodeId, eec, er);
    for (int i = 0; i < 8; ++i) {
        printf(" %02x", msef[i]);
    }
    return;
}

static uint32_t getMs(void) {
    // This is device specific. You must return the current time in miliseconds.
    // There is no inherent meaning of time "0". It can be since last boot,
    // since start of day or anything. It must just monotonically increment.

    // It is used in coSimple to detect timeouts.
    return 0;
}

void timer_irq(void) {
    // This is device specific.

    // clear interrupt
    // ...
    
    // send CANopen SYNC frame
    coSYNC(&co);
}

void can_rx_irq(void) {
    // This is device specific.

    // clear interrupt
    // ...

    // Try to receive/read PDO. The received CAN frame can be either a EMCY or
    // PDO frame. All other frame types will be discarded. This includes SDO
    // frames.
    if (0 == coRPDO(&co, CAN_ID, rpdo, &len)) {
        rpdo_received = 1;
    }

    // The CAN RX interrupt shall only be enabled during cyclic operation.
    // During cyclic operation only NMT, PDO and EMCY messages are supported. To
    // do SDO transactions disable the interrupt, do SDO calls to coSimple, and
    // then enable the interrupt again.
}
