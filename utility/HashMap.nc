
interface HashMap<k, e> {

	async command error_t insert( e *element, k key );

	async command e* get( k key );

	async command error_t remove( k key );

}