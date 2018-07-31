#include <algorithm>
#include "median_filter.hpp"

void median_filter::add_data(double new_data)
{
    data.push_front(new_data);
    data.pop_back();
}

void median_filter::clear_data()
{       
    for(int i = 0; i < data.size(); i++)
    {   
        add_data(0.0);
    }
}

double median_filter::get_result()
{       
    // Sort the data
    std::deque<double> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());
   
    // Take the middle value
    int center = (sorted_data.size() - 1) / 2;
    return sorted_data[center];
}
