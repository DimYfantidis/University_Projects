#include "word.h"

word_struct::word_struct() {
    str = nullptr;
    reps = 1;
}
word_struct::word_struct(const char *S, unsigned int R) {
    str = new char[strlen(S) + 1];
    strcpy(str,S);
    reps = R;
}
word_struct::word_struct(const word_struct &copy) {
    str = new char[strlen(copy.str) + 1];
    strcpy(str, copy.str);
    reps = copy.reps;
}


word_struct::~word_struct() {
    if (str == nullptr) {
        return;
    }
    delete[] str;
}


//SETTERS
void word_struct::set(const char *Name) {
    if (str != nullptr) {
        delete[] str;
    }
    if (Name == nullptr) {
        str = nullptr;
    } else {
        str = new char[strlen(Name) + 1];
        strcpy(str, Name);
    }
}
void word_struct::setReps(unsigned int R) {
    reps = R;
}


//GETTERS
const char *word_struct::get() const {
    return str;
}
unsigned int word_struct::getReps() const {
    return reps;
}


//ΑΛΛΕΣ ΣΥΝΑΡΤΗΣΕΙΣ
size_t word_struct::length() {
    if (str == nullptr) {
        return -1;
    }
    return strlen(str);
}
void word_struct::del() {
    set(nullptr);
}

//ΜΟΝΑΔΙΑΙΟΙ ΤΕΛΕΣΤΕΣ
word_struct &word_struct::operator ++ () {
    ++reps;
    return *this;
}
/* [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
word_struct word_struct::operator ++ (int) {
    const word_struct temp = *this;
    ++(*this);
    return temp;
}
word_struct &word_struct::operator -- () {
    --reps;
    return *this;
}
word_struct word_struct::operator -- (int) {
    const word_struct temp = *this;
    --(*this);
    return temp;
}
*/

//ΔΥΑΔΙΚΟΙ ΤΕΛΕΣΤΕΣ
word_struct &word_struct::operator = (const word_struct &arg) {
    set(arg.str);
    setReps(arg.reps);
    return *this;
}
word_struct &word_struct::operator = (const char *arg) {
    set(arg);
    return *this;
}
/* [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
char &word_struct::operator [] (size_t i) {
    return str[i];
}
*/




bool operator == (const word_struct &one, const word_struct &two) {
    if (one.get() == nullptr || two.get() == nullptr) {
        return one.get() == nullptr && two.get() == nullptr;
    }
    return strcmp(one.get(),two.get()) == 0;
}
bool operator != (const word_struct &one, const word_struct &two) {
    if (one.get() == nullptr || two.get() == nullptr) {
        return !(one.get() == nullptr && two.get() == nullptr);
    }
    return strcmp(one.get(),two.get()) != 0;
}
bool operator > (const word_struct &one, const word_struct &two) {
    return strcmp(one.get(),two.get()) > 0;
}
bool operator < (const word_struct &one, const word_struct &two) {
    return strcmp(one.get(),two.get()) < 0;
}
bool operator >= (const word_struct &one, const word_struct &two) {
    return strcmp(one.get(),two.get()) >= 0;
}
bool operator <= (const word_struct &one, const word_struct &two) {
    return strcmp(one.get(),two.get()) <= 0;
}



bool operator == (const word_struct &one, const char *two) {
    if (one.get() == nullptr || two == nullptr) {
        return one.get() == nullptr && two == nullptr;
    }
    return strcmp(one.get(),two) == 0;
}
bool operator != (const word_struct &one, const char *two) {
    if (one.get() == nullptr || two == nullptr) {
        return !(one.get() == nullptr && two == nullptr);
    }
    return strcmp(one.get(),two) != 0;
}
bool operator > (const word_struct &one, const char *two) {
    return strcmp(one.get(),two) > 0;
}
bool operator < (const word_struct &one, const char *two) {
    return strcmp(one.get(),two) < 0;
}




bool operator == (const char *one, const word_struct &two) {
    if (one == nullptr || two.get() == nullptr) {
        return one == nullptr && two.get() == nullptr;
    }
    return strcmp(one,two.get()) == 0;
}
bool operator != (const char *one, const word_struct &two) {
    if (one == nullptr || two.get() == nullptr) {
        return !(one == nullptr && two.get() == nullptr);
    }
    return strcmp(one,two.get()) != 0;
}
bool operator > (const char *one, const word_struct &two) {
    return strcmp(one,two.get()) > 0;
}
bool operator < (const char *one, const word_struct &two) {
    return strcmp(one,two.get()) < 0;
}


ostream &operator << (ostream &output, const word_struct &arg) {
    if (arg.get() != nullptr) {
        output << "(" << arg.get() << ", " << arg.getReps() << ")";
    } else {
        output << "Could not print unassigned str";
    }
    return output;
}