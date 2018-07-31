#include "fir_filter.hpp"
#include <fstream>

fir_filter::fir_filter(std::vector<double> filter_coefficients) :
    filter_coefficients(filter_coefficients)
{
    // Initialize data to the right size
    for(int i = 0; i < filter_coefficients.size(); i++) {
        this->data.push_back(0.0);
    }
}
    
fir_filter::fir_filter(double* filter_coefficients, uint8_t filter_length)
{
    for(int i = 0; i < filter_length; i++)
    {
        this->filter_coefficients.push_back(filter_coefficients[i]);
        this->data.push_back(0.0);
    }
}

fir_filter::fir_filter(std::string csv_filename)
{
    // Open file
    std::ifstream file(csv_filename);

    // Grab all filter coefficients
    while(file.good())
    {
        std::string coef_str;
        std::getline(file, coef_str, ',');
        this->filter_coefficients.push_back( stod(coef_str) );
        this->data.push_back(0.0);
    }

    // Close file
    file.close();
}

void fir_filter::clear_data()
{
    for(int i = 0; i < this->data.size(); i++)
    {
        this->add_data(0.0);
    }
}

void fir_filter::add_data(double new_data)
{
    // Append new data to the front of the list, and remove the oldest data
    this->data.push_front(new_data);
    this->data.pop_back();
}

double fir_filter::get_result()
{
    double result = 0.0;

    // Component-wise multiply the filter coefficients with the data
    for(int i = 0; i < this->filter_coefficients.size(); i++)
    {   
        result += (this->data[i]) * (this->filter_coefficients[i]);
    }

    return result;
}
