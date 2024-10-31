/**
 * @file coSimple.h
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

#ifndef __COSIMPLE_H_
#define __COSIMPLE_H_


#include <stdint.h>
#include <stddef.h>


/**
 * @brief Enable/disable setting for SYNC counter.
 *
 * The SYNC service optionally can be made to send an 1-byte counter. Each SYNC
 * the counter gets incremented by one. Before the first SYNC can be sent or
 * after communication has been stopped and restarted again the counter has to
 * be reset to 1 with coSYNCResetCounter().
 * Disabling this makes the SYNC slightly faster as no payload has to be sent.
 */
// #define CO_SYNC_COUNTER_ENABLE

#define CO_TIMEOUT_NMT (3000) //<! timeout in ms to wait for NMT response
#define CO_TIMEOUT_SDO (1000) //<! timeout in ms to wait for SDO response

/**
 * @brief Argument for coTIME
 *
 * Use this as ms argument to use the co_time_cb_t time callback function to get
 * the ms count. Alternatively the ms count can be given manually.
 */
#define CO_TIME_USE_TIMECB (UINT32_MAX)


/**
 * @brief Minimal representation of CAN frame
 *
 * identifier format assumed to be standard 11 bit
 * rtr bit always assumed to be 0 = data frame
 */
typedef struct co_msg_s {
    uint16_t cobId;  //<! CAN object identifier, 4 bit op code, 7 bit node-id
    uint8_t len;     //<! length of data
    uint8_t data[8]; //<! CAN frame data
} co_msg_t;

/**
 * @brief Callback to be implemented in application to receive raw CAN frames.
 *
 * @note Call must be non-blocking!
 *
 * @param[out] msg the received CAN frame
 * @return int -1 on error, 0 on successful reception, 1 on no data
 */
typedef int (*co_rx_cb_t)(co_msg_t *msg);

/**
 * @brief Callback to be implemented in application to send raw CAN frames.
 *
 * @param[in] msg the CAN frame to be sent
 * @return int -1 on error, 0 on successful sending
 */
typedef int (*co_tx_cb_t)(const co_msg_t *msg);

/**
 * @brief Callback to be implemented in application to handle EMCY frames.
 *
 * During normal coSimple operations EMCY frames may be received. Those are
 * forwarded to this callback and must be handled by application appropriately.
 *
 * @param nodeId the node that issued the EMCY
 * @param eec emergency error code
 * @param er error register
 * @param[in] msef manufacturer specific error field, 5 bytes
 */
typedef void (*co_emcy_cb_t)(uint8_t nodeId, uint16_t eec, uint8_t er, uint8_t *msef);

/**
 * @brief Callback to be implemented in application to get current time in ms.
 *
 * The start of time i.e. value 0 can be arbitrary and needn't have any meaning.
 * Can be time since system boot for example. The time must increase
 * monotonically though.
 *
 * @return uint32_t current time in milliseconds
 */
typedef uint32_t (*co_time_cb_t)(void);

/**
 * @brief NMT state change request type
 *
 * @see coNMTReq()
 */
typedef enum co_nmt_state_req_e {
    CO_NMT_OP = 0x01,     //<! go operational
    CO_NMT_STOP = 0x02,   //<! stop
    CO_NMT_PRE_OP = 0x80, //<! go pre-operational
    CO_NMT_RST = 0x81,    //<! do reset node
    CO_NMT_RST_COM = 0x82 //<! do reset communication
} co_nmt_state_req_t;

/**
 * @brief coSimple instance
 *
 * Fill struct with all callbacks and use reference to it in API calls.
 */
typedef struct co_s {
    co_rx_cb_t rx;     //<! application implemented callback to receive CAN frames
    co_tx_cb_t tx;     //<! application implemented callback to send CAN frames
    co_emcy_cb_t emcy; //<! application implemented callback to forward EMCY frames
    co_time_cb_t ms;   //<! application implemented callback to get current time
#ifdef CO_SYNC_COUNTER_ENABLE
    uint8_t syncCounter; //<! counter for SYNC service
#endif
} co_t;

/**
 * @brief Send NMT request to node.
 *
 * @param[in] co coSimple instance
 * @param nodeId addressed node, range 1 - 127, if = 0 then all nodes
 * @param req the state change request for the node
 * @return int -1 on error, 0 on success
 */
int coNMTReq(co_t *co, uint8_t nodeId, co_nmt_state_req_t req);

/**
 * @brief Wait until node sends boot-up message.
 *
 * @warning no timeout implemented!
 *
 * @param[in] co coSimple instance
 * @param nodeId addressed node
 * @return int -1 on error, 0 on success
 */
int coNMTWaitBoot(co_t *co, uint8_t nodeId);

/**
 * @brief Send SYNC on bus.
 *
 * @param[in] co coSimple instance
 * @return int -1 on error, 0 on success
 */
int coSYNC(co_t *co);

#ifdef CO_SYNC_COUNTER_ENABLE
/**
 * @brief Reset internal SYNC counter.
 *
 * @note This needs to be called once before calling coSYNC the first time and
 *       again after communication has been stopped and started again.
 *
 * @param[in] co coSimple instance
 * @return int -1 on error, 0 on success
 */
int coSYNCResetCounter(co_t *co);
#endif

/**
 * @brief Send Timestamp broadcast.
 *
 * @note Only milliseconds of timestamp is implemented, field for days is set to
 *       zero!
 *
 * @param[in] co coSimple instance
 * @param ms the milliceconds count to use, if CO_TIME_USE_TIMECB then the
 *           co_time_cb_t time callback is used for the actual value
 * @return int -1 on error, 0 on success
 */
int coTIME(co_t *co, uint32_t ms);

/**
 * @brief Send PDO to a node.
 *
 * @param[in] co coSimple instance
 * @param nodeId addressed node
 * @param[in] data PDO data in network byte order, LSB first!
 * @param len size of data array, range 1 - 8
 * @return int -1 on error, 0 on success
 */
int coTPDO(co_t *co, uint8_t nodeId, uint8_t *data, size_t len);

/**
 * @brief Receive PDO from a node.
 *
 * @note Call is non-blocking! Check return code.
 *
 * @param[in] co coSimple instance
 * @param nodeId addressed node
 * @param[in,out] data PDO data in network byte order, LSB first!
 *                     data array is owned by application
 * @param[out] len size of data array, range 1 - 8
 * @return int -1 on error, 0 on success, 1 on no data
 */
int coRPDO(co_t *co, uint8_t nodeId, uint8_t *data, size_t *len);

/**
 * @brief Write value to SDO server.
 *
 * @param[in] co coSimple instance
 * @param nodeId addressed node
 * @param index object dictionary index
 * @param subIndex od subindex
 * @param data value to be set
 * @param len size of data in \p data
 * @return uint32_t 0 on success, SDO abort code on error
 */
uint32_t coSDOWrite(co_t *co, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t data, size_t len);

#define coSDOWriteU32(co, nodeId, index, subIndex, data) \
    coSDOWrite(co, nodeId, index, subIndex, (uint32_t)data, sizeof(uint32_t))
#define coSDOWriteI32(co, nodeId, index, subIndex, data) \
    coSDOWrite(co, nodeId, index, subIndex, (uint32_t)data, sizeof(int32_t))
#define coSDOWriteU16(co, nodeId, index, subIndex, data) \
    coSDOWrite(co, nodeId, index, subIndex, (uint32_t)data, sizeof(uint16_t))
#define coSDOWriteI16(co, nodeId, index, subIndex, data) \
    coSDOWrite(co, nodeId, index, subIndex, (uint32_t)data, sizeof(int16_t))
#define coSDOWriteU8(co, nodeId, index, subIndex, data) \
    coSDOWrite(co, nodeId, index, subIndex, (uint32_t)data, sizeof(uint8_t))
#define coSDOWriteI8(co, nodeId, index, subIndex, data) \
    coSDOWrite(co, nodeId, index, subIndex, (uint32_t)data, sizeof(int8_t))

/**
 * @brief Read value from SDO server.
 *
 * @param[in] co coSimple instance
 * @param nodeId addressed node
 * @param index object dictionary index
 * @param subIndex od subindex
 * @param[out] data read value
 * @param len size of data in \p data
 * @return uint32_t 0 on success, SDO abort code on error
 */
uint32_t coSDORead(co_t *co, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t *data, size_t len);

#define coSDOReadU32(co, nodeId, index, subIndex, data) \
    coSDORead(co, nodeId, index, subIndex, (uint32_t *)data, sizeof(uint32_t))
#define coSDOReadI32(co, nodeId, index, subIndex, data) \
    coSDORead(co, nodeId, index, subIndex, (uint32_t *)data, sizeof(int32_t))
#define coSDOReadU16(co, nodeId, index, subIndex, data) \
    coSDORead(co, nodeId, index, subIndex, (uint32_t *)data, sizeof(uint16_t))
#define coSDOReadI16(co, nodeId, index, subIndex, data) \
    coSDORead(co, nodeId, index, subIndex, (uint32_t *)data, sizeof(int16_t))
#define coSDOReadU8(co, nodeId, index, subIndex, data) \
    coSDORead(co, nodeId, index, subIndex, (uint32_t *)data, sizeof(uint8_t))
#define coSDOReadI8(co, nodeId, index, subIndex, data) \
    coSDORead(co, nodeId, index, subIndex, (uint32_t *)data, sizeof(int8_t))


#endif /* #ifndef __COSIMPLE_H_ */
