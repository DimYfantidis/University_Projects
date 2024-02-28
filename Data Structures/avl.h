#ifndef AVL_H
#define AVL_H

#include "bst.h"

class avl: public bst {
private:
    node * insertWord(node *, const char *, bool &) override;   //Εισαγωγή λέξης σε δοσμένο κόμβο
    node * deleteWord(node *, const char *, bool &) override;   //Διαγραφή λέξης σε δοσμένο κόμβο

    node * rightRotate(node *);     //Δεξιά περιστροφή κόμβου
    node * leftRotate(node *);      //Αρισγτερή περιστροφή κόμβου

    short getBalance(node *) const;             //Παράγοντας ισορροπίας
    bool isNotBalanced(node *, short &) const;  //Έλεγχος ισορροπίας και παράγοντας ως παράμετρος με αναφορά
public:
    avl();
    ~avl();

    //Οι ακόλουθες συναρτήσεις καλούνε τις ιδιωτικές στις οποίες αντιστοιχίζονται,
    //θέτοντας ως παράμετρο τη ρίζα του δένδρου.

    bool insertWord(const char *) override;
    bool deleteWord(const char *) override;
};


#endif //AVL_H
