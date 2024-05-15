#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <opencv2/core/core.hpp>

#include "System.h"

void LoadImages(const std::string &cam0_path, const std::string &cam1_path, const std::string &timestamp_path, 
            std::vector<std::string> &img_filenames_left, std::vector<std::string> &img_filenames_right, std::vector<double> &cam_timestamps);

int main(int argc, char **argv) {
    // Input Parameter Validation
    if (argc < 5) {
        std::cerr << std::endl << "Usage: ./finalproject_vo path_to_vocabulary path_to_settings (trajectory_file_name)" << std::endl;
        return -1;
    }

    // Record filenames for all the images and timestamps and load them.
    int total_images = 0;
    std::vector<std::string> img_filenames_left;
    std::vector<std::string> img_filenames_right;
    std::vector<double> cam_timestamps;
    std::vector<float> times_track;

    std::string sequence_path(argv[3]);
    std::string timestamp_path(argv[4]);

    std::string cam0_path = sequence_path + "/mav0/cam0/data";
    std::string cam1_path = sequence_path + "/mav0/cam1/data";
    
    LoadImages(cam0_path, cam1_path, timestamp_path, img_filenames_left, img_filenames_right, cam_timestamps);

    total_images = img_filenames_left.size();
    times_track.resize(total_images);

    double t_resize = 0;
    double t_rect = 0;
    double t_track = 0;
    int num_rect = 0;
    int processed_images = 0;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

    cv::Mat img_left, img_right;
    
    for (int num_img = 0; num_img < total_images; ++num_img, ++processed_images) {
        img_left = cv::imread(img_filenames_left[num_img], cv::IMREAD_UNCHANGED);
        img_right = cv::imread(img_filenames_right[num_img], cv::IMREAD_UNCHANGED);

        if (img_left.empty()) {
            std::cerr << std::endl << "Could not read image at: "
                    << img_filenames_left[num_img] << std::endl;
            return -1;
        }

        if (img_right.empty()) {
            std::cerr << std::endl << "Could not read image at: "
                    << img_filenames_right[num_img] << std::endl;
            return -1;
        }

        double frame_time = cam_timestamps[num_img];

        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif
        // Pass the images to the SLAM system
        SLAM.TrackStereo(img_left, img_right, frame_time, vector<ORB_SLAM3::IMU::Point>(), img_filenames_left[num_img]);

        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif
        double track_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();

        times_track[num_img] = track_time;

        double T = 0;
        if(num_img < total_images - 1) {
            T = cam_timestamps[num_img + 1] - frame_time;
        } else if (num_img > 0) {
            T = frame_time - cam_timestamps[num_img - 1];
        }

        if (track_time < T) 
            usleep((T - track_time) * 1e6);
    }
    // Stop all threads

    SLAM.Shutdown();
    // Save Camera Trajectory

    return 0;
}
void LoadImages(const std::string &cam0_path, const std::string &cam1_path, const std::string &timestamp_path, 
            std::vector<std::string> &img_filenames_left, std::vector<std::string> &img_filenames_right, std::vector<double> &cam_timestamps) {
    std::ifstream time_file;
    time_file.open(timestamp_path.c_str());
    cam_timestamps.reserve(5000);
    img_filenames_left.reserve(5000);
    img_filenames_right.reserve(5000);
    std::cout << "Beginning Image Loading Process..." << std::endl;
    while(!time_file.eof()) {
        std::string s;
        getline(time_file, s);
        if(!s.empty()) {
            std::stringstream ss;
            ss << s;
            img_filenames_left.push_back(cam0_path + "/" + ss.str() + ".png");
            img_filenames_right.push_back(cam1_path + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            cam_timestamps.push_back(t/1e9);
        }
    }
    std::cout << "Finished Loading Images!" << std::endl;
}

