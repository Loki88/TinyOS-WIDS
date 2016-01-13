
#include "Wids.h"
#include "TKN154_MAC.h"


interface NetworkUtility {

	command bool isClusterHead();

	command bool isMyAddress(uint16_t *addr);

	command uint16_t* getNextHop(uint16_t *addr);

	command bool isAuthenticated(uint16_t *address);

	command void getSrcAddr(message_t *msg, uint16_t* addr);
	command void getDstAddr(message_t *msg, uint16_t* addr);

	command void getCHAddr(uint16_t* addr);
}