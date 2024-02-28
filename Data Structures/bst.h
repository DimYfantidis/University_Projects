#ifndef BST_H
#define BST_H

#include "word.h"
#include <iostream>
#include <fstream>

class node {
public:
    word_struct word;       //Κλειδί κόμβου τύπου word_struct
    unsigned short height;  //Ύψος κόμβου
    node * left;            //Αριστερό παιδί
    node * right;           //Δεξί παιδί

    node() {
        left = nullptr;
        right = nullptr;
        height = 1;
    }
    /*Ο καταστροφέας δεν είναι απαραίτητος καθώς η διαγραφή του τοπικού αντικειμένου word_struct καλεί τον
      καταστροφέα του ο οποίος αποδεσμεύει τη μνήμη της λέξης.*/
};

class bst {
protected:
    node * root;

    virtual node * insertWord(node *, const char *, bool &);    //Εισαγωγή λέξης σε δοσμένο κόμβο
    node * findWord(node *, const char *, bool &);              //Εύρεση λέξης σε δοσμένο κόμβο
    virtual node * deleteWord(node *, const char *, bool &);    //Διαγραφή λέξης σε δοσμένο κόμβο

    //Εκτύπωση στοιχείων δένδρου, δοσμένης της ρίζας.
    void inOrder(node *, ostream &) const;      //Ενδοδιατεταγμένη διάσχιση
    void preOrder(node *, ostream &) const;     //Προδιατεταγμένη διάσχιση
    void postOrder(node *, ostream &) const;    //Μεταδιατεταγμένη διάσχιση

    unsigned short height(node *) const;    //Ύψος κόμβου
    void destroyTree(node *);               //Διαγραφή δένδρου από τη μνήμη, δοσμένοης της ρίζας

    node * findMin(node *);     //Εύρεση ελαχίστου στοιχείου, δοσμένης της ρίζας.
    //node * findMax(node *); [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
public:
    bst();
    ~bst();

    //Οι ακόλουθες συναρτήσεις καλούνε τις ιδιωτικές στις οποίες αντιστοιχίζονται,
    //θέτοντας ως παράμετρο τη ρίζα του δένδρου.

    virtual bool insertWord(const char *);
    bool findWord(const char *);
    virtual bool deleteWord(const char *);

    void inOrder(ostream &) const;
    void preOrder(ostream &) const;
    void postOrder(ostream &) const;

    //void freeMemory(); [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
};

#endif //BST_H