

module SysMonDummyP {

	provides interface SystemInfo;
	provides interface NetworkUtility;

	uses interface IEEE154Frame;

} implementation {



	async command bool SystemInfo.cca() {
		return TRUE;
	}

	async command uint8_t SystemInfo.getFreeRxQueueSize() {
		return 1;
	}

	async command uint8_t SystemInfo.getLastDSN(message_t *msg) {
		return call IEEE154Frame.getDSN(msg)-1;
	}

	async command bool SystemInfo.isDetectedRemotely(wids_observable_t obs) {
		return FALSE;
	}

	async command bool NetworkUtility.isClusterHead() {
		return FALSE;
	}

	async command bool NetworkUtility.isMyAddress(uint16_t *addr) {

	}

	async command uint16_t* NetworkUtility.getNextHop(uint16_t *addr) {

	}

	async command bool NetworkUtility.isAuthenticated(uint16_t *address) {

	}
}