#ifndef _FIR_FILTER_HPP_
#define _FIR_FILTER_HPP_

#include <iostream>
#include <string>
#include <deque>
#include <vector>

#include "filter_base.hpp"

class fir_filter : public filter_base {
public:
    fir_filter(std::vector<double> filter_coefficients);
    fir_filter(double* filter_coefficients, uint8_t filter_length);
    fir_filter(std::string csv_filename);
    void add_data(double new_data);
    void clear_data();
    double get_result();
private:
    std::vector<double> filter_coefficients;
    std::deque<double> data;
};

#endif
