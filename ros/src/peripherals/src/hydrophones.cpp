#include <ros/ros.h>
#include <vector>
#include <string>
#include <serial/serial.h>
#include <math.h>

#include "peripherals/hydro_data.h"
#include "peripherals/hydro.h"
#include "peripherals/hydro_fft.h"
#include "peripherals/hydro_ffts.h"
#include "peripherals/polar_num.h"
#include "peripherals/hydro_phases.h"
#include "monitor/GetSerialDevice.h"
#include "fftw3.h"

#define NUM_HYDROPHONES (4)
#define PKT_HEADER_SIZE (12)
#define MAX_RESENDS     (3)
#define REAL            (0)
#define IMAG            (1)

using HydroDataReq = peripherals::hydro_data::Request;
using HydroDataRes = peripherals::hydro_data::Response;
using HydroFFTsReq = peripherals::hydro_ffts::Request;
using HydroFFTsRes = peripherals::hydro_ffts::Response;
using HydroPhasesReq = peripherals::hydro_phases::Request;
using HydroPhasesRes = peripherals::hydro_phases::Response;

class hydrophones
{       
public:
    hydrophones(const std::string port, int baud_rate = 115200, int timeout = 1000);
    ~hydrophones();
    std::string write(const std::string out, bool ignore_response = true, const std::string eol = "\n");
    bool get_raw_data(HydroDataReq &req, HydroDataRes &res);
    bool get_fft_data(HydroFFTsReq &req, HydroFFTsRes &res);
    bool get_hydro_phases(HydroPhasesReq &req, HydroPhasesRes &res);
private:
    bool acquire_hydro_data(std::vector<peripherals::hydro> &hydro_data);
    bool compute_fft(std::vector<peripherals::hydro_fft> &hydro_ffts);
    uint32_t stm32f4_crc32(uint8_t* data, size_t data_len, uint32_t crc = 0xFFFFFFFF);
    ros::NodeHandle nh;
    std::unique_ptr<serial::Serial> connection = nullptr;

    std::vector<fftw_plan> fftw_forward_plans;
    std::vector<double*> fftw_inputs;
    std::vector<fftw_complex*> fftw_outputs;
    uint16_t fft_size;
};

hydrophones::hydrophones(const std::string port, int baud_rate, int timeout) :
    nh(ros::NodeHandle("~"))
{       
    ROS_INFO("Connecting to hydrophones on port %s", port.c_str());
    connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t)baud_rate, serial::Timeout::simpleTimeout(timeout)));

    /* Get the packet and data sizes from launch file */
    int packet_size, data_size;
    nh.getParam("packet_size", packet_size);
    nh.getParam("data_size", data_size);

    /* Set the packet size */
    std::string out = "PS";
    out += (packet_size < 10000 ? "0" : "");
    out += (packet_size < 1000 ? "0" : "");
    out += (packet_size < 100 ? "0" : "");
    out += (packet_size < 10 ? "0" : "");
    out += std::to_string(packet_size);
    std::string p_size = write(out, false);
   
    /* Set the data size */
    out = "DS";
    out += (data_size < 10000 ? "0" : "");
    out += (data_size < 1000 ? "0" : "");
    out += (data_size < 100 ? "0" : "");
    out += (data_size < 10 ? "0" : "");
    out += std::to_string(data_size);
    std::string d_size = write(out, false);

    uint16_t packet_size_actual;
    if(p_size.length() >= 4)
    {   
        packet_size_actual = (p_size[1] << 8) | p_size[0];
        ROS_INFO("Packet Size is set to %u", packet_size_actual);
    }
   
    uint16_t data_size_actual;
    if(d_size.length() >= 4)
    {   
        data_size_actual = (d_size[1] << 8) | d_size[0];
        ROS_INFO("Data size is set to %u", data_size_actual);
    }

    uint16_t channel_size = (data_size_actual/2) / NUM_HYDROPHONES;
    this->fft_size = (channel_size/2) + 1;
    for(int i = 0; i < NUM_HYDROPHONES; i++)
    {
        fftw_inputs.push_back(new double[channel_size]);
        fftw_outputs.push_back(fftw_alloc_complex(fft_size));
        fftw_forward_plans.push_back(
            fftw_plan_dft_r2c_1d(channel_size, &fftw_inputs.back()[0], &fftw_outputs.back()[0], FFTW_MEASURE));
    }
}

hydrophones::~hydrophones()
{      
    connection->flush();
    connection->close();

    for(int i = 0; i < fftw_forward_plans.size(); i++)
    {
        fftw_destroy_plan(fftw_forward_plans[i]);
        fftw_free(fftw_outputs[i]);
        delete[] fftw_inputs[i];
    }
}

std::string hydrophones::write(const std::string out, bool ignore_response, const std::string eol)
{   
    connection->flush();
    connection->write(out + eol);
    connection->flushOutput();

    if(ignore_response) 
    {   
        return "";
    }

    return connection->readline(65536ul, eol);
}

bool hydrophones::get_hydro_phases(HydroPhasesReq &req, HydroPhasesRes &res)
{
    // Compute FFTs
    std::vector<peripherals::hydro_fft> hydro_ffts;
    if(!compute_fft(hydro_ffts))
    {
        ROS_ERROR("Failed to compute the FFT for hydro phases");
        return false;
    }

    // For each channel, find the maximum phase
    for(int i = 0; i < hydro_ffts.size(); i++)
    {
        // Find the frequency with the highest magnitude
        int max_index = 0;
        int max_magnitude = 0;
        for(int j = 0; j < hydro_ffts[i].fft.size(); j++)
        {
            if(hydro_ffts[i].fft[j].magnitude > max_magnitude)
            {
                max_index = j;
                max_magnitude = hydro_ffts[i].fft[j].magnitude;
            }
        }

        // Find the phase at that magnitude
        res.channel_phases.push_back(hydro_ffts[i].fft[max_index].phase);
    }

    return true;
}

bool hydrophones::get_raw_data(HydroDataReq &req, HydroDataRes &res)
{       
    return acquire_hydro_data(res.hydro);
}

bool hydrophones::get_fft_data(HydroFFTsReq &req, HydroFFTsRes &res)
{
    return compute_fft(res.hydro_channel);
}
    
bool hydrophones::compute_fft(std::vector<peripherals::hydro_fft> &hydro_ffts)
{
    // Acquire data from the hydrophones
    std::vector<peripherals::hydro> hydro_data;
    if(!this->acquire_hydro_data(hydro_data))
    {
        ROS_ERROR("Failed to acquire hydrophone data for FFT computation");
        return false;
    }
   
    // Compute the FFT of that data
    hydro_ffts = std::vector<peripherals::hydro_fft>(hydro_data.size());
    for(int i = 0; i < hydro_data.size(); i++)
    {
        // Copy hydrophone data into fft input buffer
        std::copy(hydro_data[i].raw_data.begin(), hydro_data[i].raw_data.end(), this->fftw_inputs[i]);
        fftw_execute(fftw_forward_plans[i]);

        // Loop through data and convert from complex representation to polar representation
        for(int j = 0; j < fft_size; j++)
        {
            // Compute magnitude and phase
            constexpr double rad_to_deg = 180.0 / 3.14159265;
            peripherals::polar_num fft_j;
            fft_j.magnitude = sqrt(pow(fftw_outputs[i][j][REAL],2) + pow(fftw_outputs[i][j][IMAG],2));
            fft_j.phase = atan2(fftw_outputs[i][j][IMAG], fftw_outputs[i][j][REAL]);

            // Add magnitude and phase to the fft data for this channel
            hydro_ffts[i].fft.push_back(fft_j);
        }
    }

    return true;
}

bool hydrophones::acquire_hydro_data(std::vector<peripherals::hydro> &hydro_data)
{  
    // Initialize the list with all the data
    for(int i = 0; i < NUM_HYDROPHONES; i++)
    {   
        peripherals::hydro data_list;
        hydro_data.push_back(data_list);
    }
    
    uint8_t header[PKT_HEADER_SIZE];
    write("ADCDR");

    uint32_t data_index = 0;
    uint8_t packet_idx = 0;
    uint8_t packet_count = 0;
    uint8_t resend_count = 0;
    do
    {  
        // Get the packet header and relevant information
        uint8_t lines;
        if((lines = connection->read(header, PKT_HEADER_SIZE)) != PKT_HEADER_SIZE)
        {       
            ROS_ERROR("Did not read enough lines! Read only %u lines.", lines);
        }
        uint32_t crc = (header[3] << 24) | (header[2] << 16) | (header[1] << 8) | header[0];
        packet_count = header[6];
        packet_idx = header[7];
        uint16_t packet_size = (header[9] << 8) | header[8];
        uint16_t total_size = (header[11] << 8) | header[10];

        //ROS_INFO("CRC: %u, Packet Index: %u, Packet Count: %u, Packet Size: %u, Total Size: %u", crc, packet_idx, packet_count, packet_size, total_size);

        // Use header to determine how much data to read
        uint8_t* packet_data = new uint8_t[packet_size];
        connection->read(packet_data, packet_size);

        // Check to see if there were any communication errors
        uint32_t host_crc = stm32f4_crc32(packet_data, packet_size);
        if(host_crc != crc)
        {      
            delete[] packet_data;
            if(resend_count < MAX_RESENDS)
            {
                ROS_ERROR("Communication error. Executing packet resend.");
                packet_idx = 0; // Do this just in case this is the last packet (get through the while loop)
                write("RETRY"); // Send the last packet again
                resend_count++;
                continue;
            }
            else
            {   
                ROS_ERROR("Communication error. Resend count exceeded maximum allowed resends. Failed to read data from hydrophones");
                return false;
            }
        }
        
        // Signal the beginning of the next packet transfer
        if(packet_idx < (packet_count - 1))
        {
            write("NEXT");
        }

        resend_count = 0;

        // Append data to output while we wait
        for(int i = 0; i < (packet_size/2); i++, data_index++)
        {   
            hydro_data[data_index % NUM_HYDROPHONES].raw_data.push_back((packet_data[(i*2)+1] << 8) | packet_data[i*2]); 
        }

        delete[] packet_data;

    } while(packet_idx < (packet_count - 1));

    connection->flush(); 

    return true;
}

uint32_t hydrophones::stm32f4_crc32(uint8_t* data, size_t data_len, uint32_t crc)
{
    // Make sure data length is a multiple of 4
    if(data_len % 4)
    {   
        ROS_ERROR("Invalid data for STM32F4 CRC. Data length must be a multiple of 4.");
        return 0;
    }

    // Compute the CRC 32 bits at a time
    for(int i = 0; i < (data_len/4); i++)
    {  
        uint32_t uint32_datum = (data[(i*4)+3] << 24) | (data[(i*4)+2] << 16) | (data[(i*4)+1] << 8) | data[(i*4)];
        crc = crc ^ uint32_datum;
        for(int j = 0; j < 32; j++)
        {      
            if(crc & 0x80000000)
            {      
                crc = (crc << 1) ^ 0x04C11DB7;
            }
            else
            {   
                crc = crc << 1;
            }
        }
    }

    return crc;
}

int main(int argc, char** argv)
{       
    ros::init(argc, argv, "hydrophones");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);


    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if(!client.call(srv)) {     
        ROS_INFO("Couldn't get \"%s\" file descriptor. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using Hydrophones on fd %s\n", srv.response.device_fd.c_str());

    hydrophones device(srv.response.device_fd.c_str());

    ros::ServiceServer raw_data_srv = nh.advertiseService("getRawData", &hydrophones::get_raw_data, &device);
    ros::ServiceServer fft_data_srv = nh.advertiseService("getFFTData", &hydrophones::get_fft_data, &device);
    ros::ServiceServer phase_data_srv = nh.advertiseService("getChannelPhases", &hydrophones::get_hydro_phases, &device);
        
    ros::spin();

    return 0;
}
