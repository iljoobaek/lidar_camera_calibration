#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream> // for read/write to file
#include <ctime>
#include <cstdint>
#include <algorithm>

#include <exception>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem; // experimental before c++17

// function from ethz-asl/kitti_to_rosbag
bool loadTimestampsIntoVector(
    const std::string &filename, std::vector<uint64_t> *timestamp_vec,
    std::vector<std::string> *ts_lines)
{
    std::ifstream import_file(filename, std::ios::in);
    if (!import_file)
    {
        return false;
    }
    timestamp_vec->clear();
    std::string line;

    std::cout << "start reading timestamp..." << std::endl;
    while (std::getline(import_file, line))
    {
        std::stringstream line_stream(line);
        //std::cout << line << std::endl;
        ts_lines->push_back(line);
        std::string timestamp_string = line_stream.str();
        std::tm t = {};
        t.tm_year = std::stoi(timestamp_string.substr(0, 4)) - 1900;
        t.tm_mon = std::stoi(timestamp_string.substr(5, 2)) - 1;
        t.tm_mday = std::stoi(timestamp_string.substr(8, 2));
        t.tm_hour = std::stoi(timestamp_string.substr(11, 2));
        t.tm_min = std::stoi(timestamp_string.substr(14, 2));
        t.tm_sec = std::stoi(timestamp_string.substr(17, 2));
        t.tm_isdst = -1;

        static const uint64_t kSecondsToNanoSeconds = 1e9;
        time_t time_since_epoch = mktime(&t);

        uint64_t timestamp = time_since_epoch * kSecondsToNanoSeconds +
                             std::stoi(timestamp_string.substr(20, 9));
        timestamp_vec->push_back(timestamp);
    }

    std::cout << "Timestamps: " << std::endl
              << timestamp_vec->front() << " " << timestamp_vec->back()
              << std::endl;

    return true;
}

bool writeTimestamps(std::string filename, std::vector<std::string> timestamps, std::vector<int> index) 
{
    std::ofstream export_file(filename, std::ios::out);
    if (!export_file)
    {
        return false;
    }
    for (int ind : index) {
        export_file << timestamps[ind] << std::endl;
    }
    export_file.close();    
}

void sync(std::vector<uint64_t> vlp_ts_vec, std::vector<uint64_t> cam_ts_vec,
    std::vector<int>& vlp_sync, std::vector<int>& cam_sync)
{
    // in our case assume lidar dara start before camera
    int curr_vlp = 0, curr_cam = 0;
    for (int i = 0; i < vlp_ts_vec.size(); i++) {
        // check if overflow or not!!
        int64_t diff = (int64_t)cam_ts_vec[0] - (int64_t)vlp_ts_vec[i]; 
        if (diff > 0) {
            curr_vlp = i;
        }   
        else break;
    }

    for (int i = curr_vlp; i < vlp_ts_vec.size(); i++) {
        for (int j = curr_cam; j < cam_ts_vec.size(); j++) {
            auto t_vlp = (int64_t)vlp_ts_vec[i] / 1000000;
            auto t_cam = (int64_t)cam_ts_vec[j] / 1000000;
            if (t_cam - t_vlp < 0) {
                curr_cam = j;
            }       
            else {
                vlp_sync.push_back(i);
                cam_sync.push_back(curr_cam);
                break;
            } 
        }
    }
    std::cout << "Before:" << std::endl;
    std::cout << "vlp size = " << vlp_sync.size() << std::endl;
    std::cout << "cam size = " << cam_sync.size() << std::endl;
    int deleted  = 0;
    for (int i = 1; i < cam_sync.size(); i++) {
        if (cam_sync[i] == cam_sync[i-1]) {
            vlp_sync.erase(vlp_sync.begin()+i-deleted);
        }
    }
    cam_sync.erase( std::unique( cam_sync.begin(), cam_sync.end() ), cam_sync.end() );
    std::cout << "------------------------------" << std::endl;
    std::cout << "After:" << std::endl;
    std::cout << "vlp size = " << vlp_sync.size() << std::endl;
    std::cout << "cam size = " << cam_sync.size() << std::endl;
}

/* save_sync_data takes sync_data_path and files vector as input
   and copy the data to the new place
*/
bool save_sync_data(std::string source_data_path, std::string target_data_path,
    std::vector<int> index) 
{ 
    std::vector<std::string> files;
    std::string path = source_data_path;
    for (const auto &entry : fs::directory_iterator(path))
    {
        //std::cout << entry.path() << std::endl;
        files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    
    fs::path targetParent = target_data_path;
    fs::create_directories(targetParent); // Recursively create target directory if not existing.
    int new_ind = 0;
    for (int ind : index) 
    {
        fs::path sourceFile = files[ind];
        fs::path targetFile = files[new_ind++];
        auto target = targetParent / targetFile.filename();
        
        //std::cout << "source: " << sourceFile << std::endl;
        //std::cout << "target: " << target << std::endl;
        try
        {
            fs::copy_file(sourceFile, target, fs::copy_options::overwrite_existing);
        }
        catch (std::exception &e)
        {
            std::cout << e.what();
        }
    }
    return true;
}

int main(int argc, char *argv[])
{

    // parameters
    std::string vlp_ts_path, vlp_data_path, cam_ts_path, cam_data_path;
    std::vector<std::string> source_path(5);
    std::vector<std::string> sync_path(5); //std::string sync_path, sync_cam_ts_path, sync_cam_data_path, sync_vlp_ts_path, sync_vlp_data_path;
    std::vector<uint64_t> vlp_ts_vec, cam_ts_vec;
    std::vector<std::string> vlp_ts, cam_ts;
    std::vector<int> vlp_sync, cam_sync; // index after synchronized

    
    if (argc > 1)
    {
        std::string fn = "../";
        fn += argv[1];
        source_path[0] = fn;
        sync_path[0] = fn + "_sync";
        source_path[1] = fn + "/velodyne_points/timestamps.txt";
        source_path[3] = fn + "/image_01/timestamps.txt";
        source_path[2] = fn + "/velodyne_points/data";
        source_path[4] = fn + "/image_01/data";
        sync_path[1] = sync_path[0] + "/velodyne_points/timestamps.txt";
        sync_path[2] = sync_path[0] + "/velodyne_points/data";
        sync_path[3] = sync_path[0] + "/image_01/timestamps.txt";
        sync_path[4] = sync_path[0] + "/image_01/data";
    }
    else
    {
        vlp_ts_path = "test.txt";
        cam_ts_path = "test.txt";
    }
    std::cout << vlp_ts_path << std::endl;
    std::cout << cam_ts_path << std::endl;

    // store timestamps from both vlp-16 and camera
    loadTimestampsIntoVector(vlp_ts_path, &vlp_ts_vec, &vlp_ts);
    loadTimestampsIntoVector(cam_ts_path, &cam_ts_vec, &cam_ts);

    // run sync algorithm
    sync(vlp_ts_vec, cam_ts_vec, vlp_sync, cam_sync);
    
    // save to new directory
    save_sync_data(source_path[2], sync_path[2], vlp_sync);
    save_sync_data(source_path[4], sync_path[4], cam_sync);
    writeTimestamps(sync_path[1], vlp_ts, vlp_sync); 
    writeTimestamps(sync_path[3], cam_ts, cam_sync); 

    std::cout << __LINE__ << __func__ << __FILE__ << std::endl;
    std::cout << __DATE__ << std::endl;
    std::cout << __TIME__ << std::endl;
}