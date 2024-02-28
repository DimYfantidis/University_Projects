#include "unorderedArray.h"
using namespace std;

// ----------------------------- PROTECTED -----------------------------

void unorderedArray::resize() {
    word_struct *temp;
    try {
        temp = new word_struct[length + ARRAY_SIZE];
    } catch (...) {
        cerr << "Not enough memory! Exiting ..." << endl;
        exit (1);
    }
    copy(data, data + length, temp);

    size_t last_diff = last - data;

    delete[] data;
    data = temp;

    last = data + last_diff;
    length += ARRAY_SIZE;
}

size_t unorderedArray::linearSearch(const char *str, bool &found) {
    if (str == nullptr) {
        cerr << "Wrong Input!" << endl;
        return 0;
    }
    size_t i;
    for (i = 0; i < currentLength(); ++i) {
        if (data[i] == str) {
            found = true;
            break;
        }
    }
    return (found ? i : 0);
}


// ----------------------------- PUBLIC -----------------------------

unorderedArray::unorderedArray() {
    try {
        data = new word_struct[ARRAY_SIZE];
    } catch (...) {
        cerr << "Not enough memory! Exiting ..." << endl;
        exit (1);
    }
    for (size_t i = 0; i < ARRAY_SIZE; ++i) {
        if ((void *)data[i].get() != nullptr) {
            cout << hex <<(void *)data[i].get() << dec << endl;
        }
    }
    last = data - 1;
    length = ARRAY_SIZE;
}

unorderedArray::~unorderedArray() {
    if (data != nullptr) {
        delete[] data;
        data = nullptr;
        last = nullptr;
        length = 0;
    }
}

size_t unorderedArray::currentLength() const {
    return last - data + 1;
}

size_t unorderedArray::maxLength() const {
    return length;
}


bool unorderedArray::findWord (const char *str, size_t &pos) {
    bool found = false;
    pos = linearSearch(str, found);
    return found;
}

bool unorderedArray::insertWord (const char *str) {
    if (str == nullptr) {
        cerr << "Wrong Input!" << endl;
        return false;
    }
    bool inserted = false;
    size_t pos;
    if (!findWord(str, pos)) {
        if (currentLength() == maxLength()) {
            resize();
        }
        ++last;
        data[currentLength() - 1] = str;
        inserted = true;
    } else {
        ++data[pos];
    }
    return inserted;
}

bool unorderedArray::deleteWord (const char *str) {
    if (str == nullptr) {
        cerr << "Wrong Input!" << endl;
        return false;
    }
    bool deleted = false;
    size_t pos;
    if (findWord(str,pos)) {
        data[pos].del();
        data[pos] = data[currentLength() - 1];
        data[currentLength() - 1].del();
        last--;
        deleted = true;
    }
    return deleted;
}

void unorderedArray::print(ostream &out) {
    for (size_t i = 0; i < last - data; ++i) {
        if (data[i].getReps() != 0) {
            out << data[i] << endl;
        }
    }
}

word_struct &unorderedArray::operator [] (size_t i) {
    return data[i];
}