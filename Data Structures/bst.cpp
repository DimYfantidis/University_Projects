#include "bst.h"

// --------------------------------- PRIVATE ---------------------------------

node * bst::findWord(node *N, const char *str, bool &found) {
    if (N == nullptr) {
        return N;
    }
    if (str < N->word) {
        return findWord(N->left, str, found);
    } else if (str > N->word) {
        return findWord(N->right, str, found);
    } else {
        found = true;
        return N;
    }
}

node * bst::insertWord(node *N, const char *str, bool &inserted) {
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
    }
    return N;
}

node * bst::deleteWord(node *N, const char *str, bool &deleted) {
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
            temp = N->right;
            N->word.del();
            delete N;
            return temp;
        } else if (N->right == nullptr) {
            temp = N->left;
            N->word.del();
            delete N;
            return temp;
        } else {
            temp = findMin(N->right);
            N->word  = temp->word;
            N->right = deleteWord(N->right, temp->word.get(), deleted);
        }
    }
    return N;
}

node * bst::findMin(node *N) {
    if (N->left == nullptr) {
        return N;
    }
    return findMin(N->left);
}
/* [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
node * bst::findMax(node *N) {
    if (N->right == nullptr) {
        return N;
    }
    return findMin(N->right);
}
*/
unsigned short bst::height(node *N) const {
    if (N == nullptr) {
        return 0;
    }
    return N->height;
}

void bst::destroyTree(node *N) {
    if (N != nullptr) {
        destroyTree(N->left);
        destroyTree(N->right);
        delete N;
    }
}

void bst::inOrder(node *N, ostream &output) const {
    if (N != nullptr) {
        inOrder(N->left, output);
        output << N->word << endl;
        inOrder(N->right, output);
    }
}

void bst::preOrder(node *N, ostream &output) const {
    if (N != nullptr) {
        output << N->word << endl;
        preOrder(N->left, output);
        preOrder(N->right, output);
    }
}

void bst::postOrder(node *N, ostream &output) const {
    if (N != nullptr) {
        postOrder(N->left, output);
        postOrder(N->right, output);
        output << N->word << endl;
    }
}

// --------------------------------- PUBLIC ---------------------------------


bst::bst() {
    root = nullptr;
}

bst::~bst() {
    destroyTree(root);
    root = nullptr;
}
/* [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
void bst::freeMemory() {
    destroyTree(root);
    root = nullptr;
}
*/

bool bst::insertWord(const char *str) {
    bool inserted = false;
    root = insertWord(root, str, inserted);
    return inserted;
}
bool bst::findWord(const char *str) {
    bool found = false;
    findWord(root, str, found);
    return found;
}
bool bst::deleteWord(const char *str) {
    bool deleted = false;
    root = deleteWord(root, str, deleted);
    return deleted;
}


void bst::inOrder(ostream &output) const {
    inOrder(root, output);
}
void bst::preOrder(ostream &output) const {
    preOrder(root, output);
}
void bst::postOrder(ostream &output) const {
    postOrder(root, output);
}