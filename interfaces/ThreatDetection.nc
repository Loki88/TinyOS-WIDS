

#include "TKN154.h"

interface ThreatDetection {

	/* 
	 * Signals the reception of a frame at MAC level.
	 */
	async event frameReceived(message_t *msg);

	/* 
	 * Signals the reception of a corrupted frame.
	 * "msg" is the corrupted message.
	 */
	async event frameRxError(message_t *msg);

	/* 
	 * Signals an error in the control procedures at level 2.
	 * "error_code" is a code identifying the error occurred.
	 */
	async event ctrlError(uint8_t error_code);

	/* 
	 * Signals the reception of a level 3 PDU
	 */
	async event packetReceived(message_t *msg);

	/*
	 * Signals an error in the authentication procedure.
	 * "msg" is 
	 */
	async event authError(message_t *msg);
}