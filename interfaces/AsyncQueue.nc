

interface AsyncQueue<t> {

  /**
   * Returns if the queue is empty.
   *
   * @return Whether the queue is empty.
   */
  async command bool empty();

  /**
   * The number of elements currently in the queue.
   * Always less than or equal to maxSize().
   *
   * @return The number of elements in the queue.
   */
  async command uint8_t size();

  /**
   * The maximum number of elements the queue can hold.
   *
   * @return The maximum queue size.
   */
  command uint8_t maxSize();

  /**
   * Get the head of the queue without removing it. If the queue
   * is empty, the return value is undefined.
   *
   * @return 't ONE' The head of the queue.
   */
  command t head();
  
  /**
   * Remove the head of the queue. If the queue is empty, the return
   * value is undefined.
   *
   * @return 't ONE' The head of the queue.
   */
  command t dequeue();

  /**
   * Enqueue an element to the tail of the queue.
   *
   * @param 't ONE newVal' - the element to enqueue
   * @return SUCCESS if the element was enqueued successfully, FAIL
   *                 if it was not enqueued.
   */
  async command error_t enqueue(t newVal);

  /**
   * Return the nth element of the queue without dequeueing it, 
   * where 0 is the head of the queue and (size - 1) is the tail. 
   * If the element requested is larger than the current queue size,
   * the return value is undefined.
   *
   * @param index - the index of the element to return
   * @return 't ONE' the requested element in the queue.
   */
  command t element(uint8_t idx);
}