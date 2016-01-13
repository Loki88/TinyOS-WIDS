
#include "Wids.h"

interface SystemInfo {

	async command bool cca();

	async command uint8_t getFreeRxQueueSize();

	async command uint8_t getLastDSN(message_t *msg, uint8_t *seq);
	
}