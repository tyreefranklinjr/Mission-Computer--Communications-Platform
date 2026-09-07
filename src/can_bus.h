/*
 * can_bus.h
 *
 * A small simulated CAN / TWAI bus.
 *
 * This is NOT a driver for real CAN or ESP32-C3 TWAI hardware. It is a
 * fixed-size ring buffer of CAN-style messages (11-bit style id, a priority
 * byte, and an 8-byte payload) that lets separate pieces of software on the
 * SAME Linux process exchange messages the way nodes on a real CAN bus
 * would: by id, with a bounded queue and no dynamic allocation. It exists
 * so the C++ mission computer simulation has something concrete to stand
 * in for "CAN/TWAI communication" without pretending to own real hardware.
 *
 * Written in plain C (not C++) and called from C++ through extern "C",
 * matching how a real TWAI driver would be a C library linked into a
 * larger C++ application.
 */

#ifndef AUTONOMY_PLATFORM_CAN_BUS_H
#define AUTONOMY_PLATFORM_CAN_BUS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_BUS_CAPACITY 64
#define CAN_PAYLOAD_BYTES 8

/* One simulated CAN frame. */
typedef struct {
    uint32_t id;                       /* message id, one per topic/sender */
    uint8_t priority;                  /* 0 = highest priority */
    uint8_t dlc;                       /* number of valid bytes in payload */
    uint8_t payload[CAN_PAYLOAD_BYTES];
} CanMessage;

/* Fixed-size ring buffer standing in for the bus itself. No malloc. */
typedef struct {
    CanMessage messages[CAN_BUS_CAPACITY];
    int head;           /* index of the oldest message */
    int count;          /* number of messages currently queued */
    int dropped_count;  /* messages lost to a full buffer or a bus fault */
    int faulted;         /* 1 while a simulated bus fault is active */
} CanBus;

/* Resets the bus to empty, healthy state. */
void CanBusInit(CanBus *bus);

/* Publishes a message onto the bus.
 * Returns 1 on success, 0 if the message was dropped (buffer full or the
 * bus is in a simulated fault state). */
int CanBusPublish(CanBus *bus, const CanMessage *message);

/* Pops the oldest queued message into *out_message.
 * Returns 1 if a message was returned, 0 if the bus was empty. */
int CanBusReceive(CanBus *bus, CanMessage *out_message);

/* Returns the number of messages currently queued. */
int CanBusPending(const CanBus *bus);

/* Simulated bus fault injection: while faulted, CanBusPublish drops every
 * message instead of queuing it. Used by the C++ fault injector to model a
 * CAN transceiver failure or a shorted bus. */
void CanBusSetFaulted(CanBus *bus, int faulted);

#ifdef __cplusplus
}
#endif

#endif /* AUTONOMY_PLATFORM_CAN_BUS_H */
