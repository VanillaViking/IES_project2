#define BUFFER_SIZE 150

// A structure to represent a queue
typedef struct Queue {
    int head; 
    int tail;
    int size;
    int capacity;
    char array[BUFFER_SIZE];
} Queue;

Queue queue_init();
char dequeue(Queue* queue);
void enqueue(Queue* queue, char item);
void enqueue_string(Queue* queue, char* str);
void enqueue_float(Queue* queue, float x, char num_digits_int, int num_digits_decimal);
int is_empty(Queue* queue);
int is_full(Queue* queue);
