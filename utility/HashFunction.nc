

interface HashFunction<key_type> {

	async command uint8_t getHash( key_type key );

	async command bool compare( key_type key1, key_type key2 );
}