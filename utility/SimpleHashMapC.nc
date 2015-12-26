


generic configuration SimpleHashMapC(typedef el_type, int n){
	provides interface HashMap<uint8_t, el_type>;
	provides interface Init;
} implementation {
	
	components new HashMapC(uint8_t, el_type, n), new SimpleHashC(n);

	HashMapC.Hash -> SimpleHashC;

	HashMap = HashMapC;
	Init = HashMapC;
}