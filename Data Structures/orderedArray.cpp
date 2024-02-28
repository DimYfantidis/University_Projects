#include "orderedArray.h"


// ----------------------------- PRIVATE -----------------------------

size_t orderedArray::binarySearch(const char *str, bool &found) {
    size_t low = 0;
    size_t high = currentLength() - 1;
    size_t mid;

    while (low <= high && high != SIZE_MAX) {
        mid = (low + high) / 2;
        if (str > data[mid]) {
            high = mid - 1;
        } else if (str < data[mid]) {
            low = mid + 1;
        } else {
            found = true;
            break;
        }
    }
    return (found ? mid : 0);
}



// ----------------------------- PUBLIC -----------------------------

orderedArray::orderedArray(): unorderedArray() {}

orderedArray::~orderedArray() {
    for (size_t i = 0; i < length; ++i) {
        data[i].del();
    }
    if (data != nullptr) {
        delete[] data;
        data = nullptr;
        last = nullptr;
        length = 0;
    }
}

bool orderedArray::findWord(const char *str, size_t &pos) {
    bool found = false;
    pos = binarySearch(str, found);
    return found;
}

bool orderedArray::insertWord(const char *str) {
    size_t pos;
    bool inserted = false;

    if (!findWord(str,pos)) {
        if (currentLength() == maxLength()) {
            resize();
        }
        ++last;
        data[currentLength() - 1] = str;

        for (size_t i = currentLength() - 1; i > 0; --i) {
            if (data[i] > data[i - 1]) {
                word_struct temp(data[i - 1]);
                data[i - 1] = data[i];
                data[i] = temp;
            } else {
                break;
            }
        }
        inserted = true;
    } else {
        ++data[pos];
    }

    return inserted;
}

bool orderedArray::deleteWord(const char *str) {
    size_t pos;
    bool deleted = false;
    if (findWord(str,pos)) {
        data[pos].del();
        for (size_t i = pos; i < currentLength() - 1; ++i) {
            data[i] = data[i + 1];
        }
        --last;
        deleted = true;
    }
    return deleted;
}