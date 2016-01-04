
#include "Wids.h"
#include "TKN154_MAC.h"


interface NetworkUtility {

	async command bool isClusterHead();

	async command bool isMyAddress(uint16_t *addr);

	async command uint16_t* getNextHop(uint16_t *addr);

	async command bool isAuthenticated(uint16_t *address);

	async command void getSrcAddr(message_t *msg, uint16_t* addr);
	async command void getDstAddr(message_t *msg, uint16_t* addr);

	async command void getCHAddr(uint16_t* addr);
}