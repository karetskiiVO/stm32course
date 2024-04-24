#ifndef IOBUFFER_h
#define IOBUFFER_h

#include <cstdint>

namespace st32 {

template <typename T, uint32_t Capacity>
class FIFObuffer {
    uint32_t size_;
    uint32_t fst, lst;

    T storage[Capacity];
public:
    FIFObuffer () : size_(0), fst(0), lst(0) {}

    void clear () {
        size_ = 0;
        fst   = 0;
        lst   = 0;
    }

    void push (T elem) {
        storage[lst] = elem;
        if (size_ == Capacity) {
            fst = (fst + 1) % Capacity;
        } else {
            size_++;
        }
        lst = (lst + 1) % Capacity;
    }

    T& front () {
        return storage[fst];
    }

    const T& front () const {
        return storage[fst];
    }

    uint32_t size () const {
        return size_;
    }

    void pop () {
        if (size_ == 0) return;
        fst = (fst + 1) % Capacity;
        size_--;
    }
};

}

#endif