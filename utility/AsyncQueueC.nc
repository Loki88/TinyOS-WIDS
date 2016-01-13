

generic module AsyncQueueC(typedef queue_t, uint8_t QUEUE_SIZE) {
  
	provides interface AsyncQueue<queue_t>;

} implementation {

	queue_t ONE_NOK queue[QUEUE_SIZE];
	uint8_t head = 0;
	uint8_t tail = 0;
	norace uint8_t size = 0;

	async command bool AsyncQueue.empty() {
		return size == 0;
	}

	async command uint8_t AsyncQueue.size() {
		return size;
	}

	command uint8_t AsyncQueue.maxSize() {
		return QUEUE_SIZE;
	}

	command queue_t AsyncQueue.head() {
		atomic return queue[head];
	}

	command queue_t AsyncQueue.dequeue() {
		queue_t t = call AsyncQueue.head();
		
		if (!call AsyncQueue.empty()) {
			head++;
			if (head == QUEUE_SIZE) 
				head = 0;
			size--;
		}
		return t;
	}

	async command error_t AsyncQueue.enqueue(queue_t newVal) {
		if (size < QUEUE_SIZE) {
			atomic{
				queue[tail] = newVal;
				tail++;
				if (tail == QUEUE_SIZE) 
					tail = 0;
				size++;
			}
			return SUCCESS;
		}
		else {
			return FAIL;
		}
	}

	command queue_t AsyncQueue.element(uint8_t idx) {
		idx += head;
		if (idx >= QUEUE_SIZE) {
			idx -= QUEUE_SIZE;
		}
		return queue[idx];
	}  

}