#ifndef _FILTER_BASE_HPP_
#define _FILTER_BASE_HPP_

class filter_base
{       
public:
    filter_base() {};
    virtual ~filter_base() {}
    virtual void add_data(double new_data) = 0;
    virtual void clear_data() = 0;
    virtual double get_result() = 0;
};

#endif
