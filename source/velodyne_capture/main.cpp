#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <sstream>
#include <iomanip>
// Include VelodyneCapture Header
#include "VelodyneCapture.h"

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem; // experimental before c++17

#include "date/tz.h"

// Get UTC time stamp
std::string get_timestamp() {
    auto tp = std::chrono::system_clock::now();
    return date::format("%F %T", tp);
}

void laser_to_cartesian(std::vector<velodyne::Laser>& lasers, std::vector<cv::Vec3f>& buffer, std::vector<float>& data) {
    buffer.resize(lasers.size());
    data.resize(lasers.size()*5);
    int idx = 0;
    for (int i = 0; i < lasers.size(); i++) {
        const double distance = static_cast<double>( lasers[i].distance );
        const double azimuth  = lasers[i].azimuth  * CV_PI / 180.0;
        const double vertical = lasers[i].vertical * CV_PI / 180.0;

        float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
        float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
        float z = static_cast<float>( ( distance * std::sin( vertical ) ) );

        if( x == 0.0f && y == 0.0f && z == 0.0f ) continue;

        float intensity = static_cast<float>(lasers[i].intensity);
        float ring = static_cast<float>(lasers[i].id);

        data[idx*5] = x;
        data[idx*5+1] = y;
        data[idx*5+2] = z;
        data[idx*5+3] = intensity;
        data[idx*5+4] = ring;

        buffer[idx] = cv::Vec3f( x, y, z );
        idx++;
    }
    buffer.resize(idx);
    data.resize(idx*5);
}

void write_bin(const std::vector<float>& data, const std::vector<std::string>& dst_paths, int idx) {
    std::stringstream ss;
    ss << dst_paths[0]  << std::setw(10) << std::setfill('0') << idx << ".bin";
    std::string fn = ss.str();
    std::cout << "Write to: "<< fn << std::endl;
    auto stream = fopen(fn.c_str(), "wb");
    fwrite(&data[0], sizeof(float), data.size(), stream);
    fclose(stream);
}

int main( int argc, char* argv[] )
{
    std::vector<std::string> dst_paths(2);
    bool fromSensor = false;
    if (argc == 2) 
    {
        fromSensor = true;
    }
    else if (argc > 3) 
    {
        std::cout << "Should come with one (from sensor) or two arguments (from pcap file): folder_name, (pcap file name)\n";
        return -1;
    }

    velodyne::VLP16Capture *capture;
    if (fromSensor) 
    {
        // Open VelodyneCapture that retrieve from Sensor
        const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.201" );
        const unsigned short port = 2368;
        capture = new velodyne::VLP16Capture ( address, port );
    }
    else 
    {
        // Open VelodyneCapture that retrieve from PCAP
        // const std::string filename = "/home/rtml/VelodyneCapture/vlp16_pcap_files/center_front_chair_test.pcap";
        const std::string filename = argv[2];
        capture = new velodyne::VLP16Capture ( filename );
    }

    if( !capture->isOpen() )
    {
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }
    std::string dst = "../../data/data_raw/";
    dst += argv[1];
    dst_paths[0] = dst + "/velodyne_points/data/";
    dst_paths[1] = dst + "/velodyne_points/timestamps.txt";

    /*
    // Create Viewer
    cv::viz::Viz3d viewer( "Velodyne" );

    // Register Keyboard Callback
    viewer.registerKeyboardCallback(
            []( const cv::viz::KeyboardEvent& event, void* cookie ){
            // Close Viewer
            if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
            static_cast<cv::viz::Viz3d*>( cookie )->close();
            }
            }
            , &viewer
            );
    */
    
    fs::path targetParent_velodyne = dst_paths[0];
    fs::create_directories(targetParent_velodyne); // Recursively create target directory if not existing.
    std::ofstream export_file(dst_paths[1], std::ios::out);

    int idx = 0; 
    // while( capture->isRun() && !viewer.wasStopped() )
    while( capture->isRun() )
    {
        auto timestamp = get_timestamp();
        
        std::vector<velodyne::Laser> lasers;
        std::vector<cv::Vec3f> buffer;
        std::vector<float> data;

        // Capture One Rotation Data
        *capture >> lasers;
        if( lasers.empty() )
        {
            continue;
        }
        laser_to_cartesian(lasers, buffer, data);
        
        // Write to binary
        write_bin(data, dst_paths, idx);        
        
        // Write time stamp
        export_file << timestamp << std::endl;

        /*
        // Create Widget
        cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_32FC3, &buffer[0] );
        cv::viz::WCloud cloud( cloudMat );

        // Show Point Cloud
        viewer.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(100));
        viewer.showWidget( "Cloud", cloud );
        viewer.spinOnce();
        */
        
        //getchar();
        idx++;
    }
    export_file.close();
    // Close All Viewers
    // cv::viz::unregisterAllWindows();

    return 0;
}
