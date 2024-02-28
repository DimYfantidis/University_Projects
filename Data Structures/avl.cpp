#include "avl.h"

// --------------------------------- PRIVATE ---------------------------------

bool avl::isNotBalanced(node *N, short &B) const {
    B = (short)(height(N->left) - height(N->right));
    return B < -1 || B > 1;
}

short avl::getBalance(node *N) const {
    return (short)(height(N->left) - height(N->right));
}

node * avl::leftRotate(node *N) {
    node *n = N->right;
    N->right = n->left;
    n->left = N;

    N->height = 1 + max(height(N->left), height(N->right));
    n->height = 1 + max(height(n->left), height(n->right));

    return n;
}

node * avl::rightRotate(node *N) {
    node *n = N->left;
    N->left = n->right;
    n->right = N;

    N->height = 1 + max(height(N->left), height(N->right));
    n->height = 1 + max(height(n->left), height(n->right));

    return n;
}

node * avl::insertWord(node *N, const char *str, bool &inserted) {
    if (N == nullptr) {
        N = new node;
        N->word = str;
        inserted = true;
        return N;
    }
    if (str < N->word) {
        N->left = insertWord(N->left, str, inserted);
    } else if (str > N->word) {
        N->right = insertWord(N->right, str, inserted);
    } else {
        ++N->word;
        return N;
    }
    N->height = 1 + max(height(N->left), height(N->right));

    short B;

    //Εξισορρόπηση
    if (isNotBalanced(N, B)) {
        if (B > 1) {
            if (str < N->left->word) {
                //Δεξιά περιστροφή
                return rightRotate(N);
            } else {
                //Διπλή περιστροφή αριστερά-δεξιά
                N->left = leftRotate(N->left);
                return rightRotate(N);
            }
        } else {
            if (str > N->right->word) {
                //Αριστερή περιστροφή
                return leftRotate(N);
            } else {
                //Διπλή περιστροφή δεξιά-αριστερά
                N->right = rightRotate(N->right);
                return leftRotate(N);
            }
        }
    }
    return N;
}

node * avl::deleteWord(node *N, const char *str, bool &deleted) {
    if (N == nullptr) {
        return N;
    }
    if (str < N->word) {
        N->left = deleteWord(N->left, str, deleted);
    } else if (str > N->word) {
        N->right = deleteWord(N->right, str, deleted);
    } else {
        deleted = true;

        node * temp;

        if (N->left == nullptr) {
            //1η περίπτωση: ο κόμβος δεν έχει παιδί ή έχει μόνο δεξί παιδί
            temp = N->right;
            N->word.del();
            delete N;
            return temp;
        } else if (N->right == nullptr) {
            //2η περίπτωση: ο κόμβος έχει μόνο αριστερό παιδί
            temp = N->left;
            N->word.del();
            delete N;
            return temp;
        } else {
            //3η περίπτωση: ο κόμβος έχει δύο παιδιά
            temp = findMin(N->right);
            N->word  = temp->word;
            N->right = deleteWord(N->right, temp->word.get(), deleted);
        }
    }

    short B;

    //Εξισορρόπηση
    if (isNotBalanced(N, B)) {
        if (B > 1) {
            if (getBalance(N->left) >= 0) {
                //Δεξιά περιστροφή
                return rightRotate(N);
            } else {
                //Διπλή περιστροφή αριστερά-δεξιά
                N->left = leftRotate(N->left);
                return rightRotate(N);
            }
        } else {
            if (getBalance(N->right) <= 0) {
                //Αριστερή περιστροφή
                return leftRotate(N);
            } else {
                //Διπλή περιστροφή δεξιά-αριστερά
                N->right = rightRotate(N->right);
                return leftRotate(N);
            }
        }
    }
    return N;
}



// --------------------------------- PUBLIC ---------------------------------


avl::avl(): bst() {}
avl::~avl() {
    destroyTree(root);
    root = nullptr;
}

bool avl::insertWord(const char *str) {
    bool inserted = false;
    root = insertWord(root, str, inserted);
    return inserted;
}

bool avl::deleteWord(const char *str) {
    bool deleted = false;
    root = deleteWord(root, str, deleted);
    return deleted;
}