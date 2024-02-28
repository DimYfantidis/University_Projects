#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <random>
#include "unorderedArray.h"
#include "orderedArray.h"
#include "bst.h"
#include "avl.h"
#include "hash_t.h"

using namespace std;

//Μακροεντολές για ευκολότερη ανάγνωση του κώδικα
#define READ_TIME chrono::high_resolution_clock::now()
#define MS_DIFF   chrono::duration_cast <chrono::milliseconds>

//Μέγεθος συνόλου Q
#define Q_SIZE    10000

//Συνάρτηση αφαίρεσης χαρακτήρα από συμβολοσειρά
void Delete (char *S, int k) {
    for (int i = k; S[i] != '\0'; ++i) {
        S[i] = S[i + 1];
    }
}

//Συνάρτηση εκτύπωση χρόνου συνάρτησης σε ρεύμα εξόδου [ΜΟΡΦΗ: (0h : 00m : 00s : 000ms)]
void functionTime (unsigned long long milliseconds, ostream &output) {
    unsigned long long seconds;
    unsigned long long minutes;
    unsigned long long hours;

    seconds = milliseconds / 1000;
    milliseconds %= 1000;
    minutes = seconds / 60;
    seconds %= 60;
    hours = minutes / 60;
    minutes %= 60;


    output << "(" << hours << "h : "
           << setw(2) << setfill('0') << minutes << "m : "
           << setw(2) << setfill('0') << seconds << "s : "
           << setw(3) << setfill('0') << milliseconds << "ms)";
}


int main() {
    ifstream inFile;                            //Ρεύμα εισόδου από αρχείο.
    ofstream outFile;                           //Ρεύμα εξόδου σε αρχείο.
    auto *scanner_string = new char[1024];      //Συμβολοσειρά μεγέθους 1KB.
    size_t pos;                                 //Δείκτης θέσης αντικειμένου σε πίνακα.

    size_t str_counter = 0;                     //Μετρητής λέξεων.
    size_t inserted_words_counter = 0;          //Μετρητής λέξεων που εισήχθησαν επιτυχώς σε μία δομή.
    size_t deleted_words_counter = 0;           //Μετρητής λέξεων που διαγράφτηκαν επιτυχώς από μία δομή.
    size_t found_words_counter = 0;             //Μετρητής λέξεων που βρέθηκαν σε μία δομή.

    long long unorderedArray_time;     //Χρόνος δημιουργίας αταξινόμητου πίνακα.
    long long orderedArray_time;       //Χρόνος δημιουργίας ταξινομημένου πίνακα.
    long long bst_time;                //Χρόνος δημιουργίας απλού δυαδικού δένδρου αναζήτησης.
    long long avl_time;                //Χρόνος δημιουργίας ισοζυγισμένου δυαδικού δένδρου.
    long long hash_time;               //Χρόνος δημιουργίας πίνακα κατακερματισμού.

    std::chrono::time_point<std::chrono::_V2::system_clock> start;      //Αντικείμενο έναρξης εγγραφής χρόνου συνάρτησης.
    std::chrono::time_point<std::chrono::_V2::system_clock> stop;       //Αντικείμενο λήξης εγγραφής χρόνου συνάρτησης.
    std::chrono::duration<long long, std::ratio<1, 1000>> duration{};   //Χρόνος διαδικασίας.

    //Γεννήτρια Mersenne Twister 19937 (64 bit) για την παραγωγή ψευδοτυχαίων αριθμών.
    std::mt19937_64 generator(time(nullptr) % 78912349211);
    std::uniform_int_distribution<int> distribution(0, INT_MAX);




    // ------------------ ΑΦΑΙΡΕΣΗ ΣΤΙΞΗΣ ΚΑΙ ΜΕΤΑΤΡΟΠΗ ΚΕΦΑΛΑΙΩΝ ΣΕ ΠΕΖΑ ------------------

    //Άνοιγμα του αρχείου ανάγνωσης κειμένου.
    inFile.open("Input.txt", ios::in);
    //Άνοιγμα του αρχείου εγγραφής λέξεων στην επεξεργασμένη τους μορφή.
    outFile.open("Modified_Input.txt", ios::out);
    if (inFile.is_open() && outFile.is_open()) {
        while (inFile >> scanner_string) {
            //Μετατροπή των κεφαλαίων χαρακτήρων της σε πεζούς και αφαίρεση στίξης.
            size_t len = strlen(scanner_string);
            for (int i = 0; i < len; ++i) {
                if (scanner_string[i] >= 'A' && scanner_string[i] <= 'Z') {
                    scanner_string[i] += 32;    //π.χ. 'D' + 32 = 'd'
                } else if (scanner_string[i] < 'a' || scanner_string[i] > 'z') {
                    //Οι ενωτικές παύλες αντικαθιστούνται με κενό, διαχωρίζοντας τη συμβολοσειρά σε περισσότερες.
                    if (scanner_string[i] == '-') {
                        scanner_string[i] = ' ';
                    }
                    //Τα υπόλοιπα σημεία στίξης διαγράφονται πλήρως χωρίς να διαχωρίζουν τη συμβολοσειρά με κενά.
                    else {
                        Delete(scanner_string, i--);
                        --len;
                    }
                }
            }
            // Δημιουργία καινούργιου αρχείου κειμένου "Modified_Input.txt",
            // ίδιο με το αρχικό χωρίς στίξη ή κεφαλαία.
            if (*scanner_string != '\0') {
                outFile << scanner_string << " ";
                //Κάθε καινούργια λέξη που διαβάζεται αυξάνει την τιμή του str_counter κατά ένα.
                //Επιπλέον ανά 13 λέξεις αλλάζει γραμμή στο Modified_Input.txt έτσι ώστε να μην γραφτούν όλες οι
                //λέξεις σε μία σειρά.
                if (++str_counter % 13 == 0)
                    outFile << "\n";
            }
        }
        inFile.close();
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }

    cout << "Words counted in text file: " << str_counter << endl;



    cout << "\nINSERTION:" << endl;


    // ------------------------------ ΑΤΑΞΙΝΟΜΗΤΟΣ ΠΙΝΑΚΑΣ ------------------------------

    unorderedArray array;

    //Δημιουργία αταξινόμητου πίνακα από αρχείο κειμένου
    inFile.open("Modified_Input.txt", ios::in);
    if (inFile.is_open()) {
        //Αρχή χρονομέτρησης δημιουργίας ταξινομημένου πίνακα.
        start = READ_TIME;
        while (inFile >> scanner_string) {
            //Εισαγωγή λέξεων στον ταξινομημένο πίνακα.
            if(array.insertWord(scanner_string)) {
                ++inserted_words_counter;
            }
        }
        //Τέλος χρονομέτρησης δημιουργίας ταξινομημένου πίνακα.
        stop = READ_TIME;
        inFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);
    //Διάρκεια εισαγωγής στοιχείων ταξινομημένου πίνακα σε milliseconds.
    unorderedArray_time = duration.count();

    //Εκτύπωση συνολικού χρόνου εισαγωγής λέξεων στην κονσόλα.
    cout << "Unordered Array Completed\t\t\t";
    functionTime(unorderedArray_time,cout);
    cout << endl;

    outFile.open("UN_ARR_Output.txt", ios::out);
    if (outFile.is_open()) {
        start = READ_TIME;
        array.print(outFile);
        stop = READ_TIME;

        outFile << endl << "Unordered Array Completed\t\t";
        functionTime(unorderedArray_time,outFile);
        outFile << endl;

        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);

    cout << "Printed " << inserted_words_counter << " elements in UN_ARR_Output.txt\t";
    functionTime(duration.count(), cout);
    cout << endl << "-------------------------------------------------------------------------" << endl;

    inserted_words_counter = 0;     //Reset.




    // ------------------------------ ΤΑΞΙΝΟΜΗΕΜΕΝΟΣ ΠΙΝΑΚΑΣ ------------------------------

    orderedArray array_ordered;

    //Δημιουργία ταξινομημένου πίνακα από αρχείο κειμένου
    inFile.open("Modified_Input.txt", ios::in);
    if (inFile.is_open()) {
        //Αρχή χρονομέτρησης δημιουργίας ταξινομημένου πίνακα.
        start = READ_TIME;
        while (inFile >> scanner_string) {
            //Εισαγωγή των λέξεων στον ταξινομημένο πίνακα.
            if(array_ordered.insertWord(scanner_string)) {
                ++inserted_words_counter;
            }
        }
        //Τέλος χρονομέτρησης δημιουργίας ταξινομημένου πίνακα.
        stop = READ_TIME;
        inFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);
    //Διάρκεια εισαγωγής στοιχείων ταξινομημένου πίνακα σε milliseconds.
    orderedArray_time = duration.count();

    //Εκτύπωση συνολικού χρόνου εισαγωγής λέξεων στην κονσόλα.
    cout << "Ordered Array Completed\t\t\t\t";
    functionTime(orderedArray_time,cout);
    cout << endl;

    outFile.open("ORD_ARR_Output.txt", ios::out);
    if (outFile.is_open()) {
        //Αρχή χρονομέτρησης εκτύπωσης ταξινομημένου πίνακα στο αρχείο ORD_ARR_Output.txt.
        start = READ_TIME;
        //Εκτύπωση συνολικού χρόνου εισαγωγής λέξεων στο αρχείο ORD_ARR_Output.txt.
        array_ordered.print(outFile);
        //Τέλος χρονομέτρησης εκτύπωσης ταξινομημένου πίνακα στο αρχείο BST_Output.txt.
        stop = READ_TIME;

        //Εκτύπωση συνολικού χρόνου εισαγωγής λέξεων στο αρχείο BST_Output.txt.
        outFile << endl << "Ordered Array Completed\t\t";
        functionTime(orderedArray_time,outFile);
        outFile << endl;

        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);

    //Εκτύπωση συνολικού χρόνου εκτύπωσης ταξινομημένου πίνακα στην κονσόλα.
    cout << "Printed " << inserted_words_counter << " elements in ORD_ARR_Output.txt\t";
    functionTime(duration.count(), cout);
    cout << endl << "-------------------------------------------------------------------------" << endl;

    inserted_words_counter = 0;




    // ------------------------------ ΔΥΑΔΙΚΟ ΔΕΝΔΡΟ ΑΝΑΖΗΤΗΣΗΣ ------------------------------

    bst tree;

    //Δημιουργία απλού δυαδικού δένδρου από αρχείο κειμένου
    inFile.open("Modified_Input.txt", ios::in);
    if (inFile.is_open()) {
        //Αρχή χρονομέτρησης δημιουργίας απλού δυαδικού δένδρου.
        start = READ_TIME;
        while (inFile >> scanner_string) {
            //Εισαγωγή των λέξεων στο απλό δυαδικό δένδρο αναζήτησης.
            if(tree.insertWord(scanner_string)) {
                ++inserted_words_counter;
            }
        }
        //Τέλος χρονομέτρησης δημιουργίας απλού δυαδικού δένδρου.
        stop = READ_TIME;
        inFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);
    //Διάρκεια εισαγωγής στοιχείων απλού δυαδικού δένδρου σε milliseconds.
    bst_time = duration.count();

    //Εκτύπωση συνολικού χρόνου εισαγωγής λέξεων στην κονσόλα.
    cout << "Binary Tree Completed\t\t\t\t";
    functionTime(bst_time,cout);
    cout << endl;


    outFile.open("BST_Output.txt", ios::out);
    if (outFile.is_open()) {
        //Αρχή χρονομέτρησης εκτύπωσης απλού δυαδικού δένδρου στο αρχείο BST_Output.txt.
        start = READ_TIME;
        //Εκτύπωση συνολικού χρόνου εισαγωγής λέξεων στο αρχείο με χρήση της προδιατεταγμένης διάσχισης.
        tree.preOrder(outFile);
        //Τέλος χρονομέτρησης εκτύπωσης απλού δυαδικού δένδρου στο αρχείο BST_Output.txt.
        stop = READ_TIME;

        //Εκτύπωση συνολικού χρόνου εισαγωγής λέξεων στο αρχείο BST_Output.txt.
        outFile << endl << "Binary Tree Completed\t\t";
        functionTime(bst_time,outFile);
        outFile << endl;

        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);

    //Εκτύπωση συνολικού χρόνου εκτύπωσης δέντρου στην κονσόλα.
    cout << "Printed " << inserted_words_counter << " elements in BST_Output.txt\t";
    functionTime(duration.count(),cout);
    cout << endl << "-------------------------------------------------------------------------" << endl;

    inserted_words_counter = 0;




    // ------------------------------ ΔΥΑΔΙΚΟ ΔΕΝΔΡΟ AVL ------------------------------

    avl avlTree;

    //Δημιουργία δυαδικού δένδρου AVL από αρχείο κειμένου
    inFile.open("Modified_Input.txt", ios::in);
    if (inFile.is_open()) {
        //Αρχή χρονομέτρησης δημιουργίας δυαδικού δένδρου AVL.
        start = READ_TIME;
        while (inFile >> scanner_string) {
            //Εισαγωγή της λέξης στο δυαδικό δένδρο AVL.
            if(avlTree.insertWord(scanner_string)) {
                ++inserted_words_counter;
            }
        }
        //Τέλος χρονομέτρησης δημιουργίας δυαδικού δένδρου AVL.
        stop = READ_TIME;
        inFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);
    //Διάρκεια εισαγωγής στοιχείων δένδρου AVL σε milliseconds.
    avl_time = duration.count();

    //Εκτύπωση συνολικού χρόνου εισαγωγής λέξεων στην κονσόλα.
    cout << "AVL Tree Completed\t\t\t\t";
    functionTime(avl_time,cout);
    cout << endl;


    outFile.open("AVL_Output.txt", ios::out);
    if (outFile.is_open()) {
        start = READ_TIME;
        avlTree.preOrder(outFile);
        stop = READ_TIME;

        outFile << endl << "AVL Tree Completed\t\t";
        functionTime(avl_time,outFile);
        outFile << endl;

        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);

    cout << "Printed " << inserted_words_counter << " elements in AVL_Output.txt\t";
    functionTime(duration.count(),cout);
    cout << endl << "-------------------------------------------------------------------------" << endl;

    inserted_words_counter = 0;




    // ------------------------------ ΠΙΝΑΚΑΣ ΚΑΤΑΚΕΡΜΑΤΙΣΜΟΥ ------------------------------

    hash_t Table;

    //Δημιουργία Hash Table από αρχείο κειμένου
    inFile.open("Modified_Input.txt", ios::in);
    if (inFile.is_open()) {
        //Αρχή χρονομέτρησης δημιουργίας Hash Table.
        start = READ_TIME;
        while (inFile >> scanner_string) {
            //Εισαγωγή της λέξης στον Hash Table.
            if(Table.insertWord(scanner_string)) {
                ++inserted_words_counter;
            }
        }
        //Τέλος χρονομέτρησης δημιουργίας Hash Table.
        stop = READ_TIME;
        inFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);
    hash_time = duration.count();

    cout << "Hash Table Completed\t\t\t\t";
    functionTime(hash_time,cout);
    cout << endl;


    outFile.open("HASH_Output.txt", ios::out);
    if (outFile.is_open()) {
        start = READ_TIME;
        Table.printTable(outFile);
        stop = READ_TIME;

        outFile << endl << "Hash Table Completed\t\t";
        functionTime(hash_time,outFile);
        outFile << endl;

        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    duration = MS_DIFF (stop - start);

    cout << "Printed " << inserted_words_counter << " elements in HASH_Output.txt\t";
    functionTime(duration.count(),cout);
    cout << endl << "-------------------------------------------------------------------------" << endl;






    /**********************************************************************/
    /****************************** ΣΥΝΟΛΟ Q ******************************/
    /**********************************************************************/

    char **Q;
    Q = new char *[Q_SIZE];


    unsigned int k = distribution(generator) % array.currentLength();
    word_struct temp(array[k]);
    for (int i = 0; i < Q_SIZE; ++i) {
        //Επιλογή τυχαίας λέξης από τον αταξινόμητο πίνακα
        k = distribution(generator) % array.currentLength();
        if (k % 10 != 0) {
            //Υπάρχει ~90% πιθανότητα το να επιλεγεί μια (πολύ πιθανόν) καινούργια λέξη στο σύνολο Q.
            //Διαφορετικά κατά ~10% θα τοποθετηθεί ξανά η ίδια λέξη.
            temp = array[k];
        }
        //Εισαγωγή της λέξης στο σύνολο Q.
        Q[i] = new char[temp.length() + 1];
        strcpy(Q[i],temp.get());
    }

    /**********************************************************************/
    /**********************************************************************/





    cout << "\nSEARCH:" << endl;

    // ------------------------------ ΑΤΑΞΙΝΟΜΗΤΟΣ ΠΙΝΑΚΑΣ ------------------------------

    start = READ_TIME;
    for (int i = 0; i < Q_SIZE; ++i) {
        if (array.findWord(Q[i], pos)) {
            ++found_words_counter;
        }
    }
    stop = READ_TIME;
    duration = MS_DIFF (stop - start);

    cout << found_words_counter << " words found in unordered array\t";
    functionTime(duration.count(), cout);
    cout << endl << "----------------------------------------------------------------" << endl;

    outFile.open("UN_ARR_Output.txt", ios::app);
    if (outFile.is_open()) {
        outFile << found_words_counter << " words found in unordered array\t";
        functionTime(duration.count(), outFile);
        outFile << endl;
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    found_words_counter = 0;




    // ------------------------------ ΤΑΞΙΝΟΜΗΜΕΝΟΣ ΠΙΝΑΚΑΣ ------------------------------

    start = READ_TIME;
    for (int i = 0; i < Q_SIZE; ++i) {
        if (array_ordered.findWord(Q[i], pos)) {
            ++found_words_counter;
        }
    }
    stop = READ_TIME;
    duration = MS_DIFF (stop - start);

    cout << found_words_counter << " words found in ordered array\t";
    functionTime(duration.count(), cout);
    cout << endl << "----------------------------------------------------------------" << endl;

    outFile.open("ORD_ARR_Output.txt", ios::app);
    if (outFile.is_open()) {
        outFile << found_words_counter << " words found in ordered array\t";
        functionTime(duration.count(), outFile);
        outFile << endl;
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    found_words_counter = 0;




    // ------------------------------ ΔΥΑΔΙΚΟ ΔΕΝΔΡΟ ΑΝΑΖΗΤΗΣΗΣ ------------------------------

    start = READ_TIME;
    for (int i = 0; i < Q_SIZE; ++i) {
        if (tree.findWord(Q[i])) {
            ++found_words_counter;
        }
    }
    stop = READ_TIME;
    duration = MS_DIFF (stop - start);

    cout << found_words_counter << " words found in binary search tree\t";
    functionTime(duration.count(), cout);
    cout << endl << "----------------------------------------------------------------" << endl;

    outFile.open("BST_Output.txt", ios::app);
    if (outFile.is_open()) {
        outFile << found_words_counter << " words found in binary search tree\t\t";
        functionTime(duration.count(), outFile);
        outFile << endl;
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    found_words_counter = 0;




    // ------------------------------ ΔΥΑΔΙΚΟ ΔΕΝΔΡΟ AVL ------------------------------

    start = READ_TIME;
    for (int i = 0; i < Q_SIZE; ++i) {
        if (avlTree.findWord(Q[i])) {
            ++found_words_counter;
        }
    }
    stop = READ_TIME;
    duration = MS_DIFF (stop - start);

    cout << found_words_counter << " words found in AVL tree\t\t";
    functionTime(duration.count(), cout);
    cout << endl << "----------------------------------------------------------------" << endl;

    outFile.open("AVL_Output.txt", ios::app);
    if (outFile.is_open()) {
        outFile << found_words_counter << " words found in AVL tree\t\t";
        functionTime(duration.count(), outFile);
        outFile << endl;
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    found_words_counter = 0;




    // ------------------------------ ΠΙΝΑΚΑΣ ΚΑΤΑΚΕΡΜΑΤΙΣΜΟΥ ------------------------------

    start = READ_TIME;
    for (int i = 0; i < Q_SIZE; ++i) {
        if (Table.findWord(Q[i], pos)) {
            ++found_words_counter;
        }
    }
    stop = READ_TIME;
    duration = MS_DIFF (stop - start);

    cout << found_words_counter << " words found in hash table\t\t";
    functionTime(duration.count(), cout);
    cout << endl << "----------------------------------------------------------------" << endl;

    outFile.open("HASH_Output.txt", ios::app);
    if (outFile.is_open()) {
        outFile << found_words_counter << " words found in hash table\t\t";
        functionTime(duration.count(), outFile);
        outFile << endl;
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }





    cout << "\nDELETION:" << endl;

    // ------------------------------ ΑΤΑΞΙΝΟΜΗΤΟΣ ΠΙΝΑΚΑΣ ------------------------------

    start = READ_TIME;
    for (int i = 0; i < Q_SIZE; ++i) {
        if (array.deleteWord(Q[i])) {
            ++deleted_words_counter;
        }
    }
    stop = READ_TIME;
    duration = MS_DIFF (stop - start);

    cout << deleted_words_counter << " words in unordered array deleted successfully\t";
    functionTime(duration.count(), cout);
    cout << endl << Q_SIZE - deleted_words_counter << " words could not be deleted"
         << endl << "---------------------------------------------------------------------------------" << endl;

    outFile.open("UN_ARR_Output.txt", ios::app);
    if (outFile.is_open()) {
        outFile << deleted_words_counter << " words in unordered array deleted successfully\t";
        functionTime(duration.count(), outFile);
        outFile << endl;
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    deleted_words_counter = 0;




    // ------------------------------ ΤΑΞΙΝΟΜΗΕΜΕΝΟΣ ΠΙΝΑΚΑΣ ------------------------------

    start = READ_TIME;
    for (int i = 0; i < Q_SIZE; ++i) {
        if (array_ordered.deleteWord(Q[i])) {
            ++deleted_words_counter;
        }
    }
    stop = READ_TIME;
    duration = MS_DIFF (stop - start);

    cout << deleted_words_counter << " words in ordered array deleted successfully\t";
    functionTime(duration.count(), cout);
    cout << endl << Q_SIZE - deleted_words_counter << " words could not be deleted"
         << endl << "---------------------------------------------------------------------------------" << endl;

    outFile.open("ORD_ARR_Output.txt", ios::app);
    if (outFile.is_open()) {
        outFile << deleted_words_counter << " words in ordered array deleted successfully\t";
        functionTime(duration.count(), outFile);
        outFile << endl;
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    deleted_words_counter = 0;




    // ------------------------------ ΔΥΑΔΙΚΟ ΔΕΝΔΡΟ ΑΝΑΖΗΤΗΣΗΣ ------------------------------

    start = READ_TIME;
    for (int i = 0; i < Q_SIZE; ++i) {
        if (tree.deleteWord(Q[i])) {
            ++deleted_words_counter;
        }
    }
    stop = READ_TIME;
    duration = MS_DIFF (stop - start);

    cout << deleted_words_counter << " words in binary search tree deleted successfully\t";
    functionTime(duration.count(), cout);
    cout << endl << Q_SIZE - deleted_words_counter << " words could not be deleted"
         << endl << "---------------------------------------------------------------------------------" << endl;

    outFile.open("BST_Output.txt", ios::app);
    if (outFile.is_open()) {
        outFile << deleted_words_counter << " words in binary search tree deleted successfully\t";
        functionTime(duration.count(), outFile);
        outFile << endl;
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }
    deleted_words_counter = 0;




    // ------------------------------ ΔΥΑΔΙΚΟ ΔΕΝΔΡΟ AVL ------------------------------

    start = READ_TIME;
    for (int i = 0; i < Q_SIZE; ++i) {
        if (avlTree.deleteWord(Q[i])) {
            ++deleted_words_counter;
        }
    }
    stop = READ_TIME;
    duration = MS_DIFF (stop - start);

    cout << deleted_words_counter << " words in AVL tree deleted successfully\t\t";
    functionTime(duration.count(), cout);
    cout << endl << Q_SIZE - deleted_words_counter << " words could not be deleted"
         << endl << "---------------------------------------------------------------------------------" << endl;

    outFile.open("AVL_Output.txt", ios::app);
    if (outFile.is_open()) {
        outFile << deleted_words_counter << " words in AVL tree deleted successfully\t";
        functionTime(duration.count(), outFile);
        outFile << endl;
        outFile.close();
    } else {
        cerr << "File Error!" << endl;
    }

    return 0;
}