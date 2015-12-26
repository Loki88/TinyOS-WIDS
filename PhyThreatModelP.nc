/*****************************************************************************************

    Copyright © 2015 Lorenzo Di Giuseppe <lorenzo.digiuseppe88@gmail.com>.
    All Rights Reserved.

    Redistribution and use in source and binary forms, with or without modification, 
    are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
    	list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, 
    	this list of conditions and the following disclaimer in the documentation 
    	and/or other materials provided with the distribution.

    3. The name of the author may not be used to endorse or promote products derived 
    	from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Lorenzo Di Giuseppe "AS IS" AND ANY EXPRESS OR IMPLIED 
    WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
    AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN 
    IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************************/

#include "WIDS.h"

module PhyThreatModelP {

	provides {
		interface RadioTx as Tx;
		interface Notify<wids_phy_observables_t> as PhyNotify;
	}

	uses {

		interface RadioRx as RxProvider;
		interface RadioTx as TxProvider;
		interface RxError;
		interface RadioCCA;
		interface IEEE154Frame;
		interface HashMapC<uint16_t, wids_rxFrame_detail> as RxMap;

	}

} implementation {

	enum {
		NO_OBSERVABLE = 0,
		CONSTANT_JAMMING = 1,
	}

	enum {
		CMD_NONE = 0,
		CMD_CCA = 1,
	};

	norace uint8_t cmd = CMD_NONE;

	ieee154_txframe_t* m_frame;
	uint32_t m_t0;
	uint32_t m_dt;

	async command error_t Tx.transmit( ieee154_txframe_t *frame, uint32_t t0, uint32_t dt ) {
		// TODO: logica per effettuare il CCA sui beacon per rilevazione di constant jamming
		if (cmd != CMD_NONE)
			return FAIL;

		if ( call IEEE154Frame.getFrameType( frame ) == 0 ) {
			m_frame = frame;
			m_t0 = t0;
			m_dt = dt;
			cmd = CMD_CCA;

			if ( call RadioCCA.request( ) == EBUSY )
				return FAIL;

			return SUCCESS;
		}
		else{
			// non è un accesso CSMA quindi non possono esserci errori di accesso al canale
			return call TxProvider.transmit( frame, t0, dt );
		}
	}


	default async event void Tx.transmitDone( ieee154_txframe_t *frame, error_t result ) { }


	async event void TxProvider.transmitDone( ieee154_txframe_t *frame, error_t result ) {
		signal Tx.transmitDone( frame, result );
	}

	async event void RadioCCA.done(error_t error) {

		cmd = CMD_NONE;

		if ( error == EBUSY ){
			signal PhyNotify.notify( CONSTANT_JAMMING );
			signal RadioTx.transmitDone( m_frame, FAIL );
		}
		else {
			error_t result;
			result = call TxProvider.transmit( m_frame, m_t0, m_dt );
			ASSERT ( result == SUCCESS );
		}

	}

	async event message_t RadioRx.received( message_t* msg ) {

		// TODO: check if an element exists inside the hash map: if it exists compare the seqno 
		// 		 and eventually update it and signal an observable, else send the message to higher layer



	}

	async event void RxError.errorOccurred( message_t* msg ) {

		// TODO: check the frame type to differentiate the error type and signal the related observable

		if ( call IEEE154Frame.getFrameType( frame ) == 2 ) { // if it is an ack
		} else {

		}

	}
}