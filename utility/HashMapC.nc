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

generic module HashMapC(typedef key_type, typedef el_type, uint8_t n) {

	provides interface HashMap<key_type, el_type>;
	provides interface Init;

	uses interface HashFunction<key_type> as Hash;
	

} implementation {

	typedef struct list {
		el_type *element;
		key_type key;
		struct list *next;
	} list_t;

	norace list_t *hashmap[n];

	uint8_t length = n;

	command error_t Init.init() {
		uint8_t i = 0;

		while( i < length ){
			hashmap[i] = NULL;
			i += 1;
		}
		return SUCCESS;
	}

	async command error_t HashMap.insert( el_type *element, key_type key ) {
		if ( call HashMap.get(key) == NULL ) {
			uint8_t i = call Hash.getHash( key );

			list_t *newEl = malloc( sizeof(list_t) );
			newEl->element = (el_type*)element;
			newEl->key = key;
			newEl->next = hashmap[i];
			hashmap[i] = newEl;

			return SUCCESS;	
		
		} else { // if an element with the key exist override it
			return FAIL;	
		}
	}

	async command el_type* HashMap.get( key_type key ) {
		uint8_t i = call Hash.getHash( key );
		list_t *tmp = hashmap[ i ];

		while ( tmp != NULL ) {
			if ( call Hash.compare(tmp->key, key ) == TRUE ){
				return tmp->element;
			}
			else 
				tmp = tmp->next;
		}

		return NULL;
	}

	async command error_t HashMap.remove( key_type key ) {

		uint8_t i = call Hash.getHash( key );
		list_t *l = hashmap[ i ];
		list_t *tmp = l;
		list_t *pre = NULL;

		while ( tmp != NULL && call Hash.compare(tmp->key, key ) == FALSE ) {
			pre = tmp;
			tmp = tmp->next;
		}

		if ( tmp == NULL )
			return FAIL;
		else {
			pre->next = tmp->next;
			return SUCCESS;
		}

	}


}