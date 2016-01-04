
#include "Wids.h"

module AnomalyDetectionP {
	
	provides interface Notify<wids_observable_t>;

	uses interface ThreatDetection;
	uses interface SystemInfo;
	uses interface NetworkUtility;
	uses interface IEEE154Frame;
	uses interface Notify<wids_observable_t> as RemoteDetection;

	uses interface Queue<wids_observable_t> as Observables;

} implementation {

	enum{
		BEACON = 0,
		DATA = 1,
		ACK = 2,
		COMMAND = 3,
		CONST_JAMM_MASK = 0x01,
		DECEP_JAMM_MASK = 0x02,
		REACT_JAMM_MASK = 0x04,

		W1 = 4,
		W2 = 3,
		W3 = 3,

		MAX_DSN_DIFF = 5,
	};

	uint8_t randomJamming = 0x00;
	wids_observable_t remoteObs = OBS_NONE;

	bool m_enabled = TRUE;

	command error_t Notify.enable() {
		m_enabled = TRUE;
	}	

	command error_t Notify.disable() {
		m_enabled = FALSE;
	}

	void evaluateRandomJamming(){

		uint8_t avg = 0;

		// The weight are not decimals to simplify the computation, it is been multiplied by 10
		avg += W1 * (randomJamming & CONST_JAMM_MASK);
		avg += W2 * (randomJamming & DECEP_JAMM_MASK);
		avg += W3 * (randomJamming & REACT_JAMM_MASK);

		if(avg > 5) {// Random jamming found
			call Observables.enqueue(OBS_4);
			// signal Notify.notify(OBS_4);
		}
	}

	async event error_t ThreatDetection.frameTransmit(message_t *msg, wids_status_t status) {
		if(m_enabled == FALSE)
			return SUCCESS;

		if(status != TX_SUCCESSFUL) { // Tx failed
			// constant jamming, link-layer jamming

			if(call IEEE154Frame.getFrameType(msg) == BEACON && status == TX_CCA_FAILED) {
				// TODO: aggiungere una soglia per la notifica dell'osservabile
				call Observables.enqueue(OBS_1);
				// signal Notify.notify(OBS_1); // Constant jamming found
				randomJamming |= CONST_JAMM_MASK;
			} else {
				randomJamming &= ~CONST_JAMM_MASK;
			}

			evaluateRandomJamming();

			if(status == TX_ACK_FAILED){
				// TODO: aggiungere una soglia per la notifica dell'osservabile
				call Observables.enqueue(OBS_5);
				// signal Notify.notify(OBS_5); // Link-layer jamming found
			}
		}

		if(m_enabled == TRUE)
				signal Notify.notify(OBS_NONE);

		return SUCCESS;
	}

	async event error_t ThreatDetection.frameReceived(message_t *msg, wids_status_t status){
		if(m_enabled == FALSE)
			return SUCCESS;

		if(status != RX_SUCCESSFUL) { // Rx failed
			// reactive jamming
			if(status == RX_CRC_FAILED){
				call Observables.enqueue(OBS_3);
				// signal Notify.notify(OBS_3); // Reactive jamming found
				randomJamming |= REACT_JAMM_MASK;
			} else {
				randomJamming &= ~REACT_JAMM_MASK;
			}

			if(status == RX_GTS_FAILED){
				call Observables.enqueue(OBS_10);
				// signal Notify.notify(OBS_10); // GTS Attack
			}

		} else {
			// deceptive jamming, backoff manipulation, replay-protection attack, GTS attack
			if(call SystemInfo.getFreeRxQueueSize() == 0){
				call Observables.enqueue(OBS_2);
				// signal Notify.notify(OBS_2); // Deceptive jamming found
				randomJamming |= DECEP_JAMM_MASK;
			} else {
				randomJamming &= ~DECEP_JAMM_MASK;
			}

			if(call IEEE154Frame.getDSN(msg) - call SystemInfo.getLastDSN(msg) > MAX_DSN_DIFF){
				call Observables.enqueue(OBS_9);
				// signal Notify.notify(OBS_9); // Replay protection attack
			}
		}

		evaluateRandomJamming();

		if(m_enabled == TRUE)
			signal Notify.notify(OBS_NONE);

		return SUCCESS;
	}

	async event error_t ThreatDetection.controlError(wids_status_t status){
		if(m_enabled == FALSE)
			return SUCCESS;

		// ACK attack
		if(status == CTRL_TIMEOUT){
			call Observables.enqueue(OBS_12);
			// signal Notify.notify(OBS_12);
		}

		if(m_enabled == TRUE)
			signal Notify.notify(OBS_NONE);

		return SUCCESS;
	}

	async event error_t ThreatDetection.packetReceived(message_t *msg, wids_status_t status){
		// selective forwarding, sinkhole, wormhole, hello flooding, sybil

		// Per il sinkhole è necessario disporre del TAKS
		uint16_t src_addr, dst_addr, ch_addr;
		if(m_enabled == FALSE)
			return SUCCESS;

		call NetworkUtility.getSrcAddr(msg, &src_addr);
		call NetworkUtility.getDstAddr(msg, &dst_addr);
		call NetworkUtility.getCHAddr(&ch_addr);

		if(*(call NetworkUtility.getNextHop(&src_addr)) == ch_addr &&
					*(call NetworkUtility.getNextHop(&dst_addr)) == ch_addr){
			call Observables.enqueue(OBS_15);
			// signal Notify.notify(OBS_15); // Wormhole detected
		}
		
		if(m_enabled == TRUE)
			signal Notify.notify(OBS_NONE);

		return SUCCESS;
	}

	event void RemoteDetection.notify(wids_observable_t obs){
		if(call NetworkUtility.isClusterHead() == TRUE){
			switch(obs){
				case OBS_1:
				case OBS_16:
					call Observables.enqueue(OBS_16);
					// signal Notify.notify(OBS_16);
					break;
				case OBS_2:
				case OBS_17:
					call Observables.enqueue(OBS_17);
					// signal Notify.notify(OBS_17);
					break;
				case OBS_3:
				case OBS_18:
					call Observables.enqueue(OBS_18);
					// signal Notify.notify(OBS_18);
					break;
				case OBS_4:
				case OBS_19:
					call Observables.enqueue(OBS_19);
					// signal Notify.notify(OBS_19);
					break;
				case OBS_5:
				case OBS_20:
					call Observables.enqueue(OBS_20);
					// signal Notify.notify(OBS_20);
					break;
				case OBS_6:
				case OBS_21:
					call Observables.enqueue(OBS_21);
					// signal Notify.notify(OBS_21);
					break;
				case OBS_7:
				case OBS_22:
					call Observables.enqueue(OBS_22);
					// signal Notify.notify(OBS_22);
					break;
				case OBS_8:
				case OBS_23:
					call Observables.enqueue(OBS_23);
					// signal Notify.notify(OBS_23);
					break;
				case OBS_9:
				case OBS_24:
					call Observables.enqueue(OBS_24);
					// signal Notify.notify(OBS_24);
					break;
				case OBS_10:
				case OBS_25:
					call Observables.enqueue(OBS_25);
					// signal Notify.notify(OBS_25);
					break;
				case OBS_11:
				case OBS_26:
					call Observables.enqueue(OBS_26);
					// signal Notify.notify(OBS_26);
					break;
				case OBS_12:
				case OBS_27:
					call Observables.enqueue(OBS_27);
					// signal Notify.notify(OBS_27);
					break;
				case OBS_13:
				case OBS_28:
					call Observables.enqueue(OBS_28);
					// signal Notify.notify(OBS_28);
					break;
				case OBS_14:
				case OBS_28:
					call Observables.enqueue(OBS_28);
					// signal Notify.notify(OBS_28);
					break;
				case OBS_15:
				case OBS_29:
					call Observables.enqueue(OBS_29);
					// signal Notify.notify(OBS_29);
					break;
				case OBS_16:
				case OBS_30:
					call Observables.enqueue(OBS_30);
					// signal Notify.notify(OBS_30);
					break;
				default:
					return;
			}
		}

		if(m_enabled == TRUE)
			signal Notify.notify(OBS_NONE);
	}

}