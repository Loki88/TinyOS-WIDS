

generic module SimpleHashC(int mod) {

	provides interface HashFunction<uint8_t> as Hash;

} implementation {

	async command uint8_t Hash.getHash( uint8_t key ) {
		return key % mod;
	}


	async command bool Hash.compare( uint8_t key1, uint8_t key2 ){
		return key1 == key2;
	}
}