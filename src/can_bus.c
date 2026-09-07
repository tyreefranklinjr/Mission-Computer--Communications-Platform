/*
 * can_bus.c
 *
 * Implementation of the simulated CAN / TWAI ring buffer described in
 * can_bus.h. Plain C, no dynamic allocation, fixed capacity.
 */

#include "can_bus.h"
#include <string.h>

void CanBusInit(CanBus *bus) {
    memset(bus, 0, sizeof(CanBus));
    bus->head = 0;
    bus->count = 0;
    bus->dropped_count = 0;
    bus->faulted = 0;
}

int CanBusPublish(CanBus *bus, const CanMessage *message) {
    if (bus->faulted) {
        bus->dropped_count = bus->dropped_count + 1;
        return 0;
    }
    if (bus->count >= CAN_BUS_CAPACITY) {
        bus->dropped_count = bus->dropped_count + 1;
        return 0;
    }

    int tail = (bus->head + bus->count) % CAN_BUS_CAPACITY;
    bus->messages[tail] = *message;
    bus->count = bus->count + 1;
    return 1;
}

int CanBusReceive(CanBus *bus, CanMessage *out_message) {
    if (bus->count == 0) {
        return 0;
    }

    *out_message = bus->messages[bus->head];
    bus->head = (bus->head + 1) % CAN_BUS_CAPACITY;
    bus->count = bus->count - 1;
    return 1;
}

int CanBusPending(const CanBus *bus) {
    return bus->count;
}

void CanBusSetFaulted(CanBus *bus, int faulted) {
    bus->faulted = faulted;
}
