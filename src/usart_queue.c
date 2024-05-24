#include "usart_queue.h"

Queue queue_init() {
    Queue queue;
    queue.capacity = BUFFER_SIZE;
    queue.head = queue.size = 0;
    queue.tail = queue.capacity - 1;

    return queue;
}

int is_full(Queue* queue)
{
    return (queue->size == queue->capacity);
}

int is_empty(Queue* queue)
{
    return (queue->size == 0);
}

// add an item to the queue.
void enqueue(Queue* queue, char item)
{
    if (is_full(queue))
        return;

    queue->tail = (queue->tail + 1) % queue->capacity;
    queue->array[queue->tail] = item;
    queue->size = queue->size + 1;
}

char dequeue(Queue* queue)
{
    if (is_empty(queue))
        return '\0';

    char item = queue->array[queue->head];
    queue->array[queue->head] = '\0';
    queue->head = (queue->head + 1) % queue->capacity;
    queue->size = queue->size - 1;
    return item;
}

void enqueue_string(Queue* queue, char* str) {
    while (*str != '\0') {
        enqueue(queue, *str);
        str++;
    }
}

void enqueue_float(Queue* queue, float x, char num_digits_int, int num_digits_decimal) {
  char num_elements = num_digits_int + num_digits_decimal + 1 + 1; // one decimal point and one null terminator
  char buffer[num_elements];

  dtostrf(x, num_elements - 1, num_digits_decimal, buffer);
  buffer[num_elements - 1] = '\0';
  enqueue_string(queue, buffer);
}
