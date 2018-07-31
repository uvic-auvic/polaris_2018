#ifndef _FIR_FILTER_HPP_
#define _FIR_FILTER_HPP_

#include <cstdint>
#include <deque>

#include "filter_base.hpp"

class median_filter : public filter_base {
public:
    median_filter(uint8_t size) : data(size) {}
    void add_data(double new_data);
    void clear_data();
    double get_result();
private:
    std::deque<double> data;
};

#endif
