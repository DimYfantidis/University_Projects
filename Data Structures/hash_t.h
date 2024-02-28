#ifndef HASH_TABLE_H
#define HASH_TABLE_H
#include "word.h"

#ifndef HASH_SIZE
#define HASH_SIZE
//Το μέγεθος του πίνακα -- πρώτος αριθμός για μείωση συγκρούσεων.
//Το κάθε μέγεθος είναι ο επόμενος πρώτος αριθμός μετά το διπλάσιο του πρηγούμενου.
//Για δέσμευση πίνακα μεγέθους μεγαλύτερου από 128002439 το μέγεθος απλά θα διπλασιάζεται.
const size_t TABLE_SIZE[] = {
          2000029,
          4000063,
          8000141,
         16000289,
         32000603,
         64001219,
        128002439
};
const unsigned int MAX_PRIME_RESIZES = sizeof(TABLE_SIZE)/sizeof(size_t) - 1;
#endif //HASH_SIZE

#define DELETED_ITEM (reinterpret_cast <word_struct *>(0xFFFFFFFFFFFF))     //Δείκτης για διεγραμμένα αντικείμενα.

class hash_t {
private:
    word_struct **table;        //Πίνακας σε δείκτες κλειδιών.
    size_t length;              //Μέγεθος πίνακα.
    size_t total_keys;          //Συνολικός αριθμός κλειδιών πίνακα.
    unsigned int resizes;       //Μετρητής μεγενθύνσεων που πραγματοποιήθηκαν στον πίνακα.

    size_t hash(const char *) const;    //Συνάρτηση hash.
    void hash2(size_t &) const;         //Συνάρτηση hash2 σε περίπτωση σύγκρουσης.
    void resize();                      //Μεγέθυνση πίνακα
public:
    hash_t();   //Κατασκευαστής πίνακα
    ~hash_t();  //Καταστροφέας πίνακα

    bool insertWord(const char *);          //Εισαγωγή κλειδιού σε πίνακα.
    //bool deleteWord(const char *); [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]    Διαγραφή κλειδιού σε πίνακα (Δεν υλοποιήθηκε).
    bool findWord(const char *, size_t &);  //Εύρεση κλειδού σε πίνακα.

    void printTable(ostream &);     //Εκτύπωση δομής σε ρεύμα εξόδου.
};


#endif //HASH_TABLE_H