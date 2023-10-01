#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

class CircularBuffer {
public:
    CircularBuffer(int size);
    void push(char* item);
    char* pop();
    bool isEmpty();
    float mm_per_rev = 17.357f;

private:
    char** buffer;
    int head;
    int tail;
    int size;
};

#endif // CIRCULARBUFFER_H