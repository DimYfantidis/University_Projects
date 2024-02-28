#ifndef ORDERED_ARRAY_H
#define ORDERED_ARRAY_H
#include "unorderedArray.h"

class orderedArray: public unorderedArray {
private:
    size_t binarySearch(const char *, bool &);  //Δυαδική αναζήτηση στοιχείου
public:
    orderedArray();
    ~orderedArray();

    bool findWord(const char *, size_t &) override; //Εύρεση λέξης
    bool insertWord(const char *) override;         //Εισαγωγή λέξης
    bool deleteWord(const char *) override;         //Διαγραφή λέξης
};


#endif //ORDERED_ARRAY_H