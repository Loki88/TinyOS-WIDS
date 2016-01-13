
#include "Wids.h"
#include "printf.h"

module AnomalyDetectionP {
	
	provides interface ObservableNotify;

	uses interface ThreatDetection;
	uses interface SystemInfo;
	uses interface NetworkUtility;
	uses interface IEEE154Frame;
	uses interface Notify<wids_observable_t> as RemoteDetection;

	uses interface AsyncQueue<wids_observable_t> as Observables;

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

	norace uint8_t randomJamming = 0x00;
	wids_observable_t remoteObs = OBS_NONE;

	wids_observable_t m_obs = OBS_NONE;

	task void enqueue(){
		call Observables.enqueue(m_obs);
		m_obs = OBS_NONE;
	}

	void evaluateRandomJamming(){

		uint8_t avg = 0;

		// The weight are not decimals to simplify the computation, it is been multiplied by 10
		avg += W1 * (randomJamming & CONST_JAMM_MASK);
		avg += W2 * (randomJamming & DECEP_JAMM_MASK);
		avg += W3 * (randomJamming & REACT_JAMM_MASK);

		if(avg > 5) {// Random jamming found
			// call Observables.enqueue(OBS_4);
			signal ObservableNotify.notify(OBS_4);
		}
	}

	async event error_t ThreatDetection.frameTransmit(message_t *msg, wids_status_t status) {

		if(status != TX_SUCCESSFUL) { // Tx failed
			// constant jamming, link-layer jamming
			uint8_t type = MHR(msg)[MHR_INDEX_FC1] & FC1_FRAMETYPE_MASK;
			if(type == FRAMETYPE_BEACON && status == TX_CCA_FAILED) {
				// TODO: aggiungere una soglia per la notifica dell'osservabile
				// call Observables.enqueue(OBS_1); // Constant jamming found
				signal ObservableNotify.notify(OBS_1);
				randomJamming |= CONST_JAMM_MASK;
			} else {
				randomJamming &= ~CONST_JAMM_MASK;
			}

			evaluateRandomJamming();

			if(status == TX_ACK_FAILED){
				// TODO: aggiungere una soglia per la notifica dell'osservabile
				// call Observables.enqueue(OBS_5); // Link-layer jamming found
				signal ObservableNotify.notify(OBS_5);
			}
		}
		else {
			signal ObservableNotify.notify(OBS_NONE);
		}

		// signal ObservableNotify.notify();

		return SUCCESS;
	}

	async event error_t ThreatDetection.frameReceived(message_t *msg, wids_status_t status){
		// printf("ThreatDetection.frameReceived\n");

		if(status != RX_SUCCESSFUL) { // Rx failed
			// reactive jamming
			if(status == RX_CRC_FAILED){
				// call Observables.enqueue(OBS_3);
				signal ObservableNotify.notify(OBS_3);
				randomJamming |= REACT_JAMM_MASK;
			} else {
				randomJamming &= ~REACT_JAMM_MASK;
			}

			if(status == RX_GTS_FAILED){
				// call Observables.enqueue(OBS_10);
				signal ObservableNotify.notify(OBS_10);
			}

		} else {
			// deceptive jamming, backoff manipulation, replay-protection attack, GTS attack
			uint8_t seq;
			bool err = FALSE;
			if(call SystemInfo.getFreeRxQueueSize() == 0){
				// call Observables.enqueue(OBS_2); // Deceptive jamming found
				signal ObservableNotify.notify(OBS_2);
				randomJamming |= DECEP_JAMM_MASK;
				err = TRUE;
			} else {
				randomJamming &= ~DECEP_JAMM_MASK;
			}
			
			if(call SystemInfo.getLastDSN(msg, &seq) == SUCCESS){
				uint16_t seqDif = 255 + MHR(msg)[MHR_INDEX_SEQNO] - seq;
				if(seqDif > 255)
					seqDif -= 255;
				if( seqDif > MAX_DSN_DIFF){
					// call Observables.enqueue(OBS_9);
					err = TRUE;
					signal ObservableNotify.notify(OBS_9);
				}
			}

			if( err == FALSE )
				signal ObservableNotify.notify(OBS_NONE);
		}

		evaluateRandomJamming();

		// signal ObservableNotify.notify();

		return SUCCESS;
	}

	async event error_t ThreatDetection.controlError(wids_status_t status){
		// printf("ThreatDetection.controlError\n");

		// ACK attack
		if(status == CTRL_TIMEOUT){
			// call Observables.enqueue(OBS_12);
			signal ObservableNotify.notify(OBS_12);
		} else {
			signal ObservableNotify.notify(OBS_NONE);
		}

		// signal ObservableNotify.notify();

		return SUCCESS;
	}

	async event error_t ThreatDetection.packetReceived(message_t *msg, wids_status_t status){
		// selective forwarding, sinkhole, wormhole, hello flooding, sybil

		// Per il sinkhole è necessario disporre del TAKS
		uint16_t src_addr, dst_addr, ch_addr;
		// printf("ThreatDetection.packetReceived\n");

		call NetworkUtility.getSrcAddr(msg, &src_addr);
		call NetworkUtility.getDstAddr(msg, &dst_addr);
		call NetworkUtility.getCHAddr(&ch_addr);

		if(*(call NetworkUtility.getNextHop(&src_addr)) == ch_addr &&
					*(call NetworkUtility.getNextHop(&dst_addr)) == ch_addr){
			// call Observables.enqueue(OBS_15); // Wormhole detected
			signal ObservableNotify.notify(OBS_15);
		} else {
			signal ObservableNotify.notify(OBS_NONE);
		}
		
		// signal ObservableNotify.notify();

		return SUCCESS;
	}

	event void RemoteDetection.notify(wids_observable_t obs){
		// printf("RemoteDetection.notify\n");
		if(call NetworkUtility.isClusterHead() == TRUE){
			switch(obs){
				case OBS_1:
				case OBS_16:
					// call Observables.enqueue(OBS_16);
					signal ObservableNotify.notify(OBS_16);
					break;
				case OBS_2:
				case OBS_17:
					// call Observables.enqueue(OBS_17);
					signal ObservableNotify.notify(OBS_17);
					break;
				case OBS_3:
				case OBS_18:
					// call Observables.enqueue(OBS_18);
					signal ObservableNotify.notify(OBS_18);
					break;
				case OBS_4:
				case OBS_19:
					// call Observables.enqueue(OBS_19);
					signal ObservableNotify.notify(OBS_19);
					break;
				case OBS_5:
				case OBS_20:
					// call Observables.enqueue(OBS_20);
					signal ObservableNotify.notify(OBS_20);
					break;
				case OBS_6:
				case OBS_21:
					// call Observables.enqueue(OBS_21);
					signal ObservableNotify.notify(OBS_21);
					break;
				case OBS_7:
				case OBS_22:
					// call Observables.enqueue(OBS_22);
					signal ObservableNotify.notify(OBS_22);
					break;
				case OBS_8:
				case OBS_23:
					// call Observables.enqueue(OBS_23);
					signal ObservableNotify.notify(OBS_23);
					break;
				case OBS_9:
				case OBS_24:
					// call Observables.enqueue(OBS_24);
					signal ObservableNotify.notify(OBS_24);
					break;
				case OBS_10:
				case OBS_25:
					// call Observables.enqueue(OBS_25);
					signal ObservableNotify.notify(OBS_25);
					break;
				case OBS_11:
				case OBS_26:
					// call Observables.enqueue(OBS_26);
					signal ObservableNotify.notify(OBS_26);
					break;
				case OBS_12:
				case OBS_27:
					// call Observables.enqueue(OBS_27);
					signal ObservableNotify.notify(OBS_27);
					break;
				case OBS_13:
				case OBS_14:
				case OBS_28:
					// call Observables.enqueue(OBS_28);
					signal ObservableNotify.notify(OBS_28);
					break;
					// call Observables.enqueue(OBS_28);
					signal ObservableNotify.notify(OBS_28);
					break;
				case OBS_15:
				case OBS_29:
					// call Observables.enqueue(OBS_29);
					signal ObservableNotify.notify(OBS_29);
					break;
				case OBS_16:
				case OBS_30:
					// call Observables.enqueue(OBS_30);
					signal ObservableNotify.notify(OBS_30);
					break;
				default:
					break;
			}
		}
		// signal ObservableNotify.notify();
		
	}

}