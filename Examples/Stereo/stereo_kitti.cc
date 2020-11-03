/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<include/sem.h>
#include<YOLO.h>


#include <torch/torch.h>
#include <chrono>


using namespace std;
using namespace std::chrono;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<string> &vstrImageRGB, vector<double> &vTimestamps);
//we need three-channel images.
int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<string> vstrImageRGB;
    vector<double> vTimestamps;
    //LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);
    LoadImages(string(argv[3]),vstrImageLeft, vstrImageRight, vstrImageRGB, vTimestamps);
    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    torch::DeviceType device_type;

    if (torch::cuda::is_available() ) {
        device_type = torch::kCUDA;
    } else {
        device_type = torch::kCPU;
    }
    torch::Device device(device_type);

    // input image size for YOLO v3
    int input_image_size = 416;

    ORB_SLAM2::Darknet net("/home/wcx/libtorch-yolov3/models/yolov3.cfg", &device);

    map<string, string> *info = net.get_net_info();

    info->operator[]("height") = std::to_string(input_image_size);

    std::cout << "loading weight ..." << endl;
    net.load_weights("/home/wcx/libtorch-yolov3/models/yolov3.weights");
    std::cout << "weight loaded ..." << endl;

    net.to(device);

    torch::NoGradGuard no_grad;
    net.eval();

    std::cout << "start to inference ..." << endl;

    cv::Mat origin_image, resized_image;
    //torch


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    cv::Mat imRGB;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_8UC1);
        imRight = cv::imread(vstrImageRight[ni],CV_8UC1);
        imRGB=cv::imread(vstrImageRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        //imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }


        vector<ORB_SLAM2::My_label> imbbx;

        origin_image=imRGB;
        resized_image=origin_image;

        cv::resize(resized_image, resized_image, cv::Size(input_image_size, input_image_size));

        cv::Mat img_float;
        resized_image.convertTo(img_float, CV_32F, 1.0/255);

        auto img_tensor = torch::from_blob(img_float.data, {1, input_image_size, input_image_size, 3}).to(device);
        img_tensor = img_tensor.permute({0,3,1,2});

        auto start = std::chrono::high_resolution_clock::now();

        auto output = net.forward(img_tensor);


        auto result = net.write_results(output, 80, 0.6, 0.4);

        auto end = std::chrono::high_resolution_clock::now();

        auto duration = duration_cast<milliseconds>(end - start);

        if(result.dim() >1)
        {
            int obj_num = result.size(0);

            float w_scale = float(origin_image.cols) / input_image_size;
            float h_scale = float(origin_image.rows) / input_image_size;

            result.select(1,1).mul_(w_scale);
            result.select(1,2).mul_(h_scale);
            result.select(1,3).mul_(w_scale);
            result.select(1,4).mul_(h_scale);

            auto result_data = result.accessor<float, 2>();

            for (int i = 0; i < result.size(0) ; i++)
            {
                ORB_SLAM2::My_label label;
                //cv::rectangle(origin_image, cv::Point(result_data[i][1], result_data[i][2]), cv::Point(result_data[i][3], result_data[i][4]), cv::Scalar(0, 0, 255), 1, 1, 0);
                label.label_class = result_data[i][7];
                label.position[0] = (result_data[i][1] + result_data[i][3])/2;
                label.position[1] = (result_data[i][2] + result_data[i][4])/2;
                label.position[2] = abs(result_data[i][3]-result_data[i][1]);
                label.position[3] = abs(result_data[i][4]-result_data[i][2]);
                label.confidence =  result_data[i][6];
                imbbx.push_back(label);
            }

    }
        //torch


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        SLAM.TrackStereo_bbx(imLeft,imRight,tframe,imbbx);


        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}
/*
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
*/
void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector<string> &vstrImageRGB,
                vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";
    //额外多读彩色图做目标检测
    string strPrefixRGB = strPathToSequence + "/image_2/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    vstrImageRGB.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
        vstrImageRGB[i] = strPrefixRGB + ss.str() + ".png";
    }
}
