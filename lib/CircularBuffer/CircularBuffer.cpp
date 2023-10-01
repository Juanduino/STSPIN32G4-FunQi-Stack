#include "CircularBuffer.h"

float mm_per_rev = 17.357f;

CircularBuffer::CircularBuffer(int size) : size(size), head(0), tail(0) {
    buffer = new char*[size];
}

void CircularBuffer::push(char* item) {
    buffer[head] = item;
    head = (head + 1) % size;
}

char* CircularBuffer::pop() {
    if (isEmpty()) {
        return nullptr;
    }
    char* item = buffer[tail];
    tail = (tail + 1) % size;
    return item;
}

bool CircularBuffer::isEmpty() {
    return head == tail;
}