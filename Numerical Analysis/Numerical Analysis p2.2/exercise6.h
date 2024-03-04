#ifndef EXERCISE6_H
#define EXERCISE6_H

#include <vector>
#include <utility>


double integralTrapezoid(const std::vector<std::pair<double, double>>& samples)
{
    double I;
    int N_trapezoids = (int)samples.size() - 1;
    int last = (int)samples.size() - 1;
    double a = samples[0].first;
    double b = samples[last].first;
    double sum = 0.0;

    for (int i = 1; i < samples.size() - 1; ++i) {
        sum += samples[i].second;
    }

    I = (b - a) / (2 * N_trapezoids) * (samples[0].second + samples[last].second + 2 * sum);
    return I;
}

double integralSimpson(const std::vector<std::pair<double, double>>& samples)
{
    double I;
    int N = (int)samples.size() - 1;
    int last = (int)samples.size() - 1;
    double a = samples[0].first;
    double b = samples[last].first;
    double sum1 = 0.0;
    double sum2 = 0.0;

    for (int i = 1; i <= N/2 - 1; ++i) {
        sum1 += samples[2 * i].second;
    }
    for (int i = 1; i <= N/2; ++i) {
        sum2 += samples[2 * i - 1].second;
    }

    I = (b - a) / (3 * N) * (samples[0].second + samples[last].second + 2 * sum1 + 4 * sum2);
    return I;
}

#endif //EXERCISE6_H
