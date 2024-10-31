/**
 * @file coSimple.c
 * @author Niklaus Leuenberger <@NikLeberg>
 * @brief Simple CANopen master functionality - coSimple.
 * @version 0.3
 * @date 2023-06-23
 *
 * @copyright Copyright (c) 2024 Niklaus Leuenberger
 *            SPDX-License-Identifier: MIT
 *
 * Minimalistic and simple to use CANopen master. Implements no object directory
 * and is for sure not CiA301 compliant. But it allows the control of CANopen
 * nodes on an extreme constrained bare metal environments. For more fully
 * fledged masters with automatic configuration based on dcf files see other
 * libraries like LeLy-core from LeLy industries.
 *
 * This library implements:
 * - NMT master
 * - SYNC producer
 * - EMCY receiver
 * - TIME producer
 * - PDO receive/transmit
 *    => only one PDO for each
 * - SDO client
 *    => only expedited
 *    => only on default channels
 *    => only at max 4 byte data types, (u)int8 - (u)int32
 *
 * Mode of operation:
 * - reset/reboot node with NMT
 * - configure node with SDO service
 * - bring node in operational state with NMT
 * - cyclically:
 *   - send PDO to node         @see coTPDO()
 *   - issue SYNC               @see coSYNC()
 *   - receive PDO from node    @see coRPDO()
 *   => received EMCY messages in this cyclic mode are forwarded to application
 *   => no SDO transactions supported in cyclic operation!
 *
 */


#include "coSimple.h"
#include <assert.h>
#include <string.h> // memcpy


/**
 * @brief COB-IDs for default communication channels.
 *
 * Not all are implemented!
 */
typedef enum co_cob_id_e {
    COB_ID_NMT = 0x000,   //<! NMT node control
    COB_ID_SYNC = 0x080,  //<! Sync
    COB_ID_EMCY = 0x080,  //<! Emergency (+ node id)
    COB_ID_TIME = 0x100,  //<! Timestamp
    COB_ID_TPDO1 = 0x180, //<! first TxPDO (+ node id)
    COB_ID_RPDO1 = 0x200, //<! first RxPDO (+ node id)
    // COB_ID_TPDO2 = 0x280, //<! second TxPDO (+ node id), not supported
    // COB_ID_RPDO2 = 0x300, //<! second RxPDO (+ node id), not supported
    // COB_ID_TPDO3 = 0x380, //<! third TxPDO (+ node id), not supported
    // COB_ID_RPDO3 = 0x400, //<! third RxPDO (+ node id), not supported
    // COB_ID_TPDO4 = 0x480, //<! fourth TxPDO (+ node id), not supported
    // COB_ID_RPDO4 = 0x500, //<! fourth RxPDO (+ node id), not supported
    COB_ID_TSDO = 0x580, //<! transmit SDO
    COB_ID_RSDO = 0x600, //<! receive SDO
    COB_ID_HRTB = 0x700, //<! heartbeat (+ node id)
    // COB_ID_TLSS = 0x7e4,  //<! layer setting service, transmit, not supported
    // COB_ID_RLSS = 0x7e5,  //<! layer setting service, receive, not supported
} co_cob_id_t;


/**
 * @brief Get the COB-Id type of the message.
 *
 * @param[in] msg message to check
 * @return co_cob_id_t evaluated COB-ID
 */
static inline co_cob_id_t getCOBIDType(const co_msg_t *msg);

/**
 * @brief Get the nodeId of the message.
 *
 * @note nodeId can only be retreived for COB-IDs: EMCY, xPDO, xSDO and HRTB
 *
 * @param[in] msg message to check
 * @return uint8_t evaluated nodeId
 */
static inline uint8_t getNodeId(const co_msg_t *msg);

/**
 * @brief Check for timeout.
 *
 * @param[in] co coSimple instance
 * @param start start point in ms from where the timeout began running
 * @param timeout timeout in ms to check against, must not be bigger than UINT32_MAX / 2
 * @return int -1 on timeout, 0 on no timeout
 */
static inline int haveTimeout(co_t *co, uint32_t start, const uint32_t timeout);


int coNMTReq(co_t *co, uint8_t nodeId, co_nmt_state_req_t req) {
    assert(co);
    assert(co->tx);
    assert(nodeId <= 127); // nodeId is allowed to be zero
    // prepare CAN frame
    co_msg_t msg = {
        .cobId = COB_ID_NMT, // NMT node control
        .len = 2,
        .data = {req /* requested state */, nodeId /* addressed node */}};
    // send CAN frame
    return co->tx(&msg);
}

int coNMTWaitBoot(co_t *co, uint8_t nodeId) {
    assert(co);
    assert(co->rx);
    assert(co->ms);
    assert(nodeId > 0 && nodeId <= 127);
    // wait blocking for response but with timeout
    uint32_t start = co->ms();
    int ret;
    co_msg_t msg;
    while (-1 != (ret = co->rx(&msg))) {
        // check if this was the frame we are looking for
        if (COB_ID_HRTB == getCOBIDType(&msg) // received frame was a boot up message (heartbeat)
            && nodeId == getNodeId(&msg)      // was from the requested node
            && 1 == msg.len                   // has exactly one byte of data
            && 0x00 == msg.data[0]) {         // data has NMT state of 0 = boot-up
            return 0;                         // all good, got boot-up message
        }
        // check for timeout
        if (0 != haveTimeout(co, start, CO_TIMEOUT_NMT)) {
            return -1; // timeout
        }
    }
    return ret; // forward error of rx callback
}

int coSYNC(co_t *co) {
    assert(co);
    assert(co->tx);
    // prepare CAN frame
#ifndef CO_SYNC_COUNTER_ENABLE
    co_msg_t msg = {
        .cobId = COB_ID_SYNC, // Sync
        .len = 0
        // note: SYNC frame doesn't carry any data
    };
#else
    co_msg_t msg = {
        .cobId = COB_ID_SYNC, // Sync
        .len = 1,
        .data = {co->syncCounter}};
    // increment SYNC counter
    ++(co->syncCounter);
#endif
    // send CAN frame
    return co->tx(&msg);
}

#ifdef CO_SYNC_COUNTER_ENABLE
int coSYNCResetCounter(co_t *co) {
    assert(co);
    co->syncCounter = 1;
    return 0; // no error
}
#endif

int coTIME(co_t *co, uint32_t ms) {
    assert(co);
    assert(co->tx);
    assert(co->ms);
    // get ms counter
    // does user provide a specific millisecond time value?
    if (CO_TIME_USE_TIMECB == ms) {
        // no, get our own
        ms = co->ms();
    }
    // prepare CAN frame
    // ToDo: implement day field of TIME_OF_DAY data type
    co_msg_t msg = {
        .cobId = COB_ID_TIME, // TimeStamp
        .len = 6,
        .data = {
            ms & 0xff, (ms >> 8) & 0xff, (ms >> 16) & 0xff, // miliseconds 24bit
            0,                                              // reserved 8bits
            0, 0},                                          // days 16bits, not implemented!
    };
    // send CAN frame
    return co->tx(&msg);
}

int coTPDO(co_t *co, uint8_t nodeId, uint8_t *data, size_t len) {
    assert(co);
    assert(co->tx);
    assert(nodeId > 0 && nodeId <= 127);
    assert(data);
    assert(len > 0 && len <= 8);
    // prepare CAN frame
    co_msg_t msg = {
        .cobId = COB_ID_RPDO1 + nodeId, // Master Tx, Slave Rx
        .len = len};
    // copy data
    memcpy(msg.data, data, len);
    // send CAN frame
    return co->tx(&msg);
}

int coRPDO(co_t *co, uint8_t nodeId, uint8_t *data, size_t *len) {
    assert(co);
    assert(co->rx);
    assert(co->emcy);
    assert(nodeId > 0 && nodeId <= 127);
    assert(data);
    assert(len);
    // try to receive a CAN frame
    co_msg_t msg;
    int ret = co->rx(&msg);
    if (0 != ret) {
        // either no data or error, forward to application
        return ret;
    }
    // we have received something, process it
    co_cob_id_t cobId = getCOBIDType(&msg);
    uint8_t rxNodeId = getNodeId(&msg);
    if (COB_ID_EMCY == cobId) {
        // emergency, assemble fields and forward to application
        uint32_t eec = msg.data[0] | (msg.data[1] << 8);
        co->emcy(rxNodeId, eec, msg.data[2], &msg.data[3]);
        return 1;
    } else if (COB_ID_TPDO1 == cobId && nodeId == rxNodeId) {
        // our looked for PDO, copy data to application
        *len = msg.len;
        memcpy(data, msg.data, msg.len);
        return 0;
    }
    // not something we can handle, tell application no data was received
    return 1;
}

uint32_t coSDOWrite(co_t *co, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t data, size_t len) {
    assert(co);
    assert(co->tx);
    assert(co->rx);
    assert(co->ms);
    assert(nodeId > 0 && nodeId <= 127);
    assert(len > 0 && len <= 4); // at max (u)int32_t supported!
    // prepare CAN frame
    uint8_t nField = ((4 - len) << 2); // count of unused bytes of data part
    co_msg_t msg = {
        .cobId = COB_ID_RSDO + nodeId, // receive SDO channel
        .len = 8,
        .data = {
            // client command specifier, SDO client download initiate, expedited, 4 - len unused bytes
            0x23 | nField,
            index & 0xff /* index LSB */, (index >> 8) & 0xff /* index MSB */,
            subIndex,
            // data, LSB first!
            data & 0xff,
            1 < len ? (data >> 8) & 0xff : 0x00,
            2 < len ? (data >> 16) & 0xff : 0x00,
            3 < len ? (data >> 24) & 0xff : 0x00}};
    // send CAN frame
    int ret = co->tx(&msg);
    if (0 != ret) {
        return ret; // error while sending
    }
    // wait blocking for response but with timeout
    uint32_t start = co->ms();
    while (-1 != (ret = co->rx(&msg))) {
        // check if this was the frame we are looking for
        if (COB_ID_TSDO == getCOBIDType(&msg)       // received frame was a SDO response
            && nodeId == getNodeId(&msg)            // was from the requested node
            && 8 == msg.len                         // has exactly 8 bytes of data
            && (index & 0xff) == msg.data[1]        // requested index, low byte
            && ((index >> 8) & 0xff) == msg.data[2] // requested index, high byte
            && subIndex == msg.data[3]) {           // requested subindex
            // this is for us, check status code
            switch (msg.data[0] & 0xf0) {
            case 0x20: // ok, finish
            case 0x60:
                return 0;
            case 0x80: // error
            default:
                return -1;
            }
        }
        // check for timeout
        if (0 != haveTimeout(co, start, CO_TIMEOUT_SDO)) {
            return -1; // timeout
        }
    }
    return ret; // forward error of rx callback
}

uint32_t coSDORead(co_t *co, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t *data, size_t len) {
    assert(co);
    assert(co->tx);
    assert(co->rx);
    assert(co->ms);
    assert(nodeId > 0 && nodeId <= 127);
    assert(data);
    assert(len > 0 && len <= 4); // at max (u)int32_t supported!
    // prepare CAN frame
    co_msg_t msg = {
        .cobId = COB_ID_RSDO + nodeId, // receive SDO channel
        .len = 8,
        .data = {
            0x40, // client command specifier, SDO client upload initiate
            index & 0xff /* index LSB */, (index >> 8) & 0xff /* index MSB */,
            subIndex
            // no data
        }};
    // send CAN frame
    int ret = co->tx(&msg);
    if (0 != ret) {
        return ret; // error while sending
    }
    // wait blocking for response but with timeout
    uint32_t start = co->ms();
    uint8_t nField = ((4 - len) << 2); // count of unused bytes of data part
    while (-1 != (ret = co->rx(&msg))) {
        // check if this was the frame we are looking for
        if (COB_ID_TSDO == getCOBIDType(&msg)       // received frame was a SDO response
            && nodeId == getNodeId(&msg)            // was from the requested node
            && 8 == msg.len                         // has exactly 8 bytes of data
            && (index & 0xff) == msg.data[1]        // requested index, low byte
            && ((index >> 8) & 0xff) == msg.data[2] // requested index, high byte
            && subIndex == msg.data[3]              // requested subindex
            && 0x03 == (msg.data[0] & 0x03)         // e[1]=1, s[0]=1
            && nField == (msg.data[0] & 0x0c)) {    // n[3:2]=count of unused bytes
            // this is for us, check status code
            switch (msg.data[0] & 0xf0) {
            case 0x40: // ok, return
            case 0x60:
                *data = msg.data[4] |
                        ((1 < len) ? (msg.data[5] << 8) : 0x00) |
                        ((2 < len) ? (msg.data[6] << 16) : 0x00) |
                        ((3 < len) ? (msg.data[7] << 24) : 0x00);
                return 0;
            case 0x80: // error
            default:
                return -1;
            }
        }
        // check for timeout
        if (0 != haveTimeout(co, start, CO_TIMEOUT_SDO)) {
            return -1; // timeout
        }
    }
    return ret; // forward error of rx callback
}


static inline co_cob_id_t getCOBIDType(const co_msg_t *msg) {
    assert(msg);
    // COB-ID type is in the first four bits, the function code
    return (msg->cobId & 0b11110000000);
}

static inline uint8_t getNodeId(const co_msg_t *msg) {
    assert(msg);
    // nodeId is in the last seven bits
    return (msg->cobId & 0b00001111111);
}

// Source: https://stackoverflow.com/a/3167693 user @nategoose
static inline int haveTimeout(co_t *co, uint32_t start, const uint32_t timeout) {
    assert(co);
    assert(timeout < (UINT32_MAX / 2));
    uint32_t now = co->ms();
    int32_t delta = now - start;
    uint32_t duration = delta < 0 ? -delta : delta; // abs(delta)
    if (duration >= timeout) {
        return -1; // timeout
    } else {
        return 0; // ok
    }
}
