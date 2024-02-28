#ifndef WORD_H
#define WORD_H

#include <iostream>
#include <algorithm>
#include <cstring>
#include <climits>

using namespace std;

class word_struct {
protected:
    char *str;
    unsigned int reps;
public:
    word_struct();
    word_struct(const char *, unsigned int);
    word_struct(const word_struct &);
    ~word_struct();

    void set(const char *);
    void setReps(unsigned int);

    const char *get() const;
    unsigned int getReps() const;

    size_t length();
    void del();

    word_struct &operator =  (const word_struct &);
    word_struct &operator =  (const char *);
    word_struct &operator ++ ();
    //word_struct operator ++ (int);    [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
    //word_struct &operator -- ();      [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
    //word_struct operator -- (int);    [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
    //char &operator [] (size_t);       [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
};

bool operator == (const word_struct &, const word_struct &);
bool operator != (const word_struct &, const word_struct &);
bool operator >  (const word_struct &, const word_struct &);
bool operator <  (const word_struct &, const word_struct &);
bool operator >= (const word_struct &, const word_struct &);
bool operator <= (const word_struct &, const word_struct &);
bool operator == (const word_struct &, const char *);
bool operator != (const word_struct &, const char *);
bool operator >  (const word_struct &, const char *);
bool operator <  (const word_struct &, const char *);
bool operator == (const char *, const word_struct &);
bool operator != (const char *, const word_struct &);
bool operator >  (const char *, const word_struct &);
bool operator <  (const char *, const word_struct &);

ostream &operator << (ostream &, const word_struct &);

/* [[ΑΧΡΗΣΙΜΟΠΟΙΗΤΟΣ ΚΩΔΙΚΑΣ]]
#include <chrono>
class Timer {
private:
    std::chrono::time_point<std::chrono::_V2::system_clock> start;
    std::chrono::time_point<std::chrono::_V2::system_clock> stop;
    std::chrono::duration<long long, std::ratio<1, 1000000000>> duration{};
public:
    Timer() {
        start = std::chrono::high_resolution_clock::now();
    }
    ~Timer() {
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast <std::chrono::nanoseconds> (stop - start);
        if (duration.count() > 0) {
            std::cout << duration.count() << " nanoseconds\n";
        }
    }
};
*/
#endif //WORD_H