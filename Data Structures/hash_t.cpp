#include "hash_t.h"
#include <cmath>

// PRIVATE

size_t hash_t::hash(const char *str) const {
    size_t sum = 0;
    auto len = (int)strlen(str);
    for (int i = 0; i < len; i++) {
        sum += (str[i] - 80) * pow(100,len - i - 1);
    }
    return sum % length;
}

void hash_t::hash2(size_t &index) const {
    index = (index + 1) % length;
}

void hash_t::resize() {
    word_struct **temp;
    size_t old_length = length;

    try {
        if (resizes <= MAX_PRIME_RESIZES) {
            temp = new word_struct *[TABLE_SIZE[resizes]];
            length = TABLE_SIZE[resizes];
        } else {
            temp = new word_struct *[2 * length];
            length *= 2;
        }
    } catch (...) {
        cerr << "Not enough memory! Exiting ..." << endl;
        exit (1);
    }

    for (size_t i = 0; i < length; i++) {
        temp[i] = nullptr;
    }
    size_t index;
    for (size_t i = 0; i < old_length; i++) {
        if (table[i] != nullptr && table[i] != DELETED_ITEM) {
            index = hash(table[i]->get());
            while(temp[index] != nullptr) {
                hash2(index);
            }
            temp[index] = new word_struct;
            *temp[index] = *table[i];
            table[i]->del();
            delete table[i];
        }
    }
    delete[] table;

    table = new word_struct *[length];
    for (size_t i = 0; i < length; i++) {
        table[i] = temp[i];
    }
    delete[] temp;
}

// PUBLIC

hash_t::hash_t() {
    try {
        table = new word_struct *[TABLE_SIZE[0]];
    } catch (...) {
        cerr << "Not enough memory! Exiting ..." << endl;
        exit (1);
    }
    length = TABLE_SIZE[0];

    for (size_t i = 0; i < length; i++) {
        table[i] = nullptr;
    }
    total_keys = 0;
    resizes = 0;
}

hash_t::~hash_t() {
    for (size_t i = 0; i < length; i++) {
        if (table[i] != nullptr && table[i] != DELETED_ITEM) {
            delete table[i];
        }
    }
    delete[] table;
}

bool hash_t::insertWord(const char *str) {
    if (total_keys != 0 && (double)length / total_keys < 1.3) {
        resize();
    }
    size_t index = hash(str);
    while (table[index] != nullptr && table[index] != DELETED_ITEM) {
        if (*table[index] == str) {
            ++(*table[index]);
            return false;
        }
        hash2(index);
    }
    table[index] = new word_struct;
    *table[index] = str;
    total_keys++;
    return true;
}


bool hash_t::findWord(const char *str, size_t &pos) {
    size_t index = hash(str);
    while (table[index] != nullptr) {
        if (table[index] != DELETED_ITEM && *table[index] == str) {
            pos = index;
            return true;
        } else {
            hash2(index);
        }
    }
    return false;
}

void hash_t::printTable(ostream &output) {
    for (size_t i = 0; i < length; i++) {
        if (table[i] != nullptr && table[i] != DELETED_ITEM) {
            output << *table[i] << endl;
        }
    }
}