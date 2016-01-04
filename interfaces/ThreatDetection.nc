
#include "Wids.h"

interface ThreatDetection {

	/* 
	 * Signals the transmission of a frame and the status of the transmission.
	 */
	async event error_t frameTransmit(message_t *msg, wids_status_t status);

	/* 
	 * Signals the reception of a frame with athe associated status.
	 */
	async event error_t frameReceived(message_t *msg, wids_status_t status);

	/* 
	 * Signals an error in the control procedures at level 2.
	 * "error_code" is a code identifying the error occurred.
	 */
	async event error_t controlError(wids_status_t status);

	/* 
	 * Signals the reception of a level 3 PDU with the associated status
	 */
	async event error_t packetReceived(message_t *msg, wids_status_t status);
}