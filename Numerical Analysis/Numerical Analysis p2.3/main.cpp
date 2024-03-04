#include <iostream>
#include <vector>
#include <utility>

#include "exercise7.h"
#include "utils.h"


// https://www.capital.gr/finance/historycloses/cener
// https://www.capital.gr/finance/historycloses/cnlcap
// Closest to birthday (07/05/2002): 07/05/2022 -> 2,7950
int main()
{
    using std::vector;
    using std::pair;

    /*
     *  Casting date to number explained:
     *  We choose one specific date as the origin (in this project: 20/04/2021).
     *  The origin date corresponds top zero value (20/04/2021 -> 0.0).
     *  All of the other dates correspond to their difference from the origin in days.
     *
     *  e.g.: for origin 20/04/2021 then
     *          18/04/2021 corresponds to -2.0
     *          23/04/2021 corresponds to  3.0
     *          06/05/2021 corresponds to 16.0
     *
     *  Thus, DD/MM/YYYY corresponds to x -> f(x) = close on DD/MM/YYYY
     */

    // (x, f(x)) pairs: x = DD/MM/YY cast to double, f(x) = close on that day
    vector<pair<double, double>> CENER_CLOSES;
    vector<pair<double, double>> CNLCAP_CLOSES;

    // The five next days cast to double
    vector<double> CENER_NEXT_DAYS;
    vector<double> CNLCAP_NEXT_DAYS;

    // Auxiliary variables for the predicted closes
    vector<vector<double> *> *predicted_closes;
    vector<double> *pred;

    // Auxiliary function for preventing the command prompt from closing when running in cmd.
    std::atexit([] () {
        std::cout << '\n' << "Press \"Enter\" to exit ..." << std::endl;
        std::cin.get();
    });

    // Initializes the 10 previous closes from 07/05/2021
    init_closes("CENER.txt", CENER_CLOSES);
    init_closes("CNLCAP.txt", CNLCAP_CLOSES);


    polynomial *LS_2[] = {
            leastSquares(2, CENER_CLOSES),
            leastSquares(2, CNLCAP_CLOSES)
    };

    polynomial *LS_3[] = {
            leastSquares(3, CENER_CLOSES),
            leastSquares(3, CNLCAP_CLOSES)

    };
    polynomial *LS_4[] = {
            leastSquares(4, CENER_CLOSES),
            leastSquares(4, CNLCAP_CLOSES)
    };


    // [1]: Predicting the closes of CENER.

    std::cout << "[1]: Calculated Polynomials for CENER\n";
    std::cout << "2nd: y = " << *LS_2[0] << '\n';
    std::cout << "3rd: y = " << *LS_3[0] << '\n';
    std::cout << "4th: y = " << *LS_4[0] << '\n';
    std::cout << std::endl;


    init_next_days("CENER NEXT.txt", CENER_NEXT_DAYS);
    init_next_days("CNLCAP NEXT.txt", CNLCAP_NEXT_DAYS);


    predicted_closes = new vector<vector<double> *>;

    for (double day: CENER_NEXT_DAYS)
    {
        pred = new vector<double>(3);

        (*pred)[0] = LS_2[0]->getValue(day);
        (*pred)[1] = LS_3[0]->getValue(day);
        (*pred)[2] = LS_4[0]->getValue(day);

        predicted_closes->push_back(pred);
    }

    std::cout << "EXPECTED CLOSES FOR CENER\n";
    for (vector<double> *close: *predicted_closes) {
        std::cout << "{2nd: " << (*close)[0] << ", 3rd: " << (*close)[1] << ", 4th: " << (*close)[2] << "}\n";
    }
    std::flush(std::cout);


    for (vector<double> *close: *predicted_closes) {
        delete close;
    }
    predicted_closes->clear();

    std::cout << '\n' << std::endl;

    // [2]: Predicting the closes of CNLCAP.

    std::cout << "[2]: Calculated Polynomials for CNLCAP\n";
    std::cout << "2nd: y = " << *LS_2[1] << '\n';
    std::cout << "3rd: y = " << *LS_3[1] << '\n';
    std::cout << "4th: y = " << *LS_4[1] << '\n';
    std::cout << std::endl;

    for (double day: CNLCAP_NEXT_DAYS)
    {
        pred = new vector<double>(3);

        (*pred)[0] = LS_2[1]->getValue(day);
        (*pred)[1] = LS_3[1]->getValue(day);
        (*pred)[2] = LS_4[1]->getValue(day);

        predicted_closes->push_back(pred);
    }

    std::cout << "EXPECTED CLOSES FOR CNLCAP\n";
    for (vector<double> *close: *predicted_closes) {
        std::cout << "{2nd: " << (*close)[0] << ", 3rd: " << (*close)[1] << ", 4th: " << (*close)[2] << "}\n";
    }
    std::flush(std::cout);

    for (vector<double> *close: *predicted_closes) {
        delete close;
    }
    delete predicted_closes;

    for (int i = 0; i < 2; ++i) {
        delete LS_2[i];
        delete LS_3[i];
        delete LS_4[i];
    }

    return EXIT_SUCCESS;
}
