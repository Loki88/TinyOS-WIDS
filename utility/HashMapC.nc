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

#include "UTIL.h"

generic module HashMapC(typedef key_type, typedef el_type, int n) {

	provides interface HashMap<key_type, el_type>;
	provides interface Init;

	uses interface HashFunction<key_type> as Hash;
	

} implementation {

	list_t *hashmap[n];

	command error_t Init.init() {
		uint8_t i = 0;
		while( i<n ){
			hashmap[i] = NULL;
			i += 1;
		}
	}

	async command error_t HashMap.insert( el_type *element, key_type key ) {
		uint8_t index = call Hash.getHash( key );
		list_t *tmp = hashmap[ index ];
		list_t newEl;

		newEl.element = (void*) element;
		newEl.key = (void*) &key;
		newEl.next = tmp;

		hashmap[index] = &newEl;

		return SUCCESS;
	}

	async command el_type* HashMap.get( key_type key ) {
		uint8_t index = call Hash.getHash( key );
		list_t *tmp = hashmap[ index ];

		while ( tmp->next != NULL && call Hash.compare(*((key_type*) tmp->key), key ) == FALSE ) {
			tmp = tmp->next;
		}

		return tmp->element;
	}

	async command error_t HashMap.remove( key_type key ) {

		uint8_t index = call Hash.getHash( key );
		list_t *l = hashmap[ index ];
		list_t *tmp = l;
		list_t *prev = NULL;

		while ( tmp != NULL && call Hash.compare(*((key_type*) tmp->key), key ) == FALSE ) {
			prev = tmp;
			tmp = tmp->next;
		}

		if ( tmp == NULL )
			return FAIL;
		else {
			prev->next = tmp->next;
			return SUCCESS;
		}

	}


}