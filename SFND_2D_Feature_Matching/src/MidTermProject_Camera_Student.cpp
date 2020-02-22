/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;


// detector function
double Detector(string detectorType, vector<cv::KeyPoint> &keypoints, cv::Mat &imgGray, bool bVis){
    //// -> SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    double t;
  	if (detectorType.compare("SHITOMASI") == 0)
    {
      t = detKeypointsShiTomasi(keypoints, imgGray, bVis);
    }
    else if (detectorType.compare("HARRIS") == 0)
    {
      t = detKeypointsHarris(keypoints, imgGray, bVis);
    }
    else
    {
      t = detKeypointsModern(keypoints, imgGray, detectorType, bVis);
    }
  
  	return t;
}

// Focuse on Keypoints of preceding vehicle only
void FocusOnVehicle(bool bFocusOnVehicle, cv::Rect vehicleRect, vector<cv::KeyPoint> &keypoints){
    vector<cv::KeyPoint> keypoints_cropped;
    if (bFocusOnVehicle)
    {
      int x_min = vehicleRect.x;
      int x_max = vehicleRect.x + vehicleRect.width;
      int y_min = vehicleRect.y;
      int y_max = vehicleRect.y + vehicleRect.height;
      //Erase the keypoints which are out of the  defined rectangular box
      for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
      {
        if ((it->pt.x < x_max) && (it->pt.x > x_min) && (it->pt.y < y_max) && (it->pt.y > y_min)){
          keypoints_cropped.push_back(*it);
        }
        else{
          continue;
        }
      }
      keypoints = keypoints_cropped;
    } 
}

// Match Function
void Match(vector<DataFrame> &dataBuffer, string matcherType, string descriptorType, string selectorType)
{
  //// MATCH KEYPOINT DESCRIPTORS ////
  vector<cv::DMatch> matches;
  //string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
  //string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
  //string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

  matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                   (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                   matches, descriptorType, matcherType, selectorType);

  // store matches in current data frame
  (dataBuffer.end() - 1)->kptMatches = matches; 
  cout<< "Number of matched keypoints identified for the preceding vehicle:" << matches.size() << endl;
}

// Load Data into buffer
void LoadData(int imgFillWidth, int imgStartIndex, int imgIndex, string imgBasePath, string imgPrefix, string imgFileType, DataFrame &frame, vector<DataFrame> &dataBuffer, int dataBufferSize )
{
  //// LOAD IMAGE INTO BUFFER ////
  // assemble filenames for current index
  ostringstream imgNumber;
  imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
  string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
  cout << "**" << imgFullFilename << endl;

  // load image from file and convert to grayscale
  cv::Mat img, imgGray;
  img = cv::imread(imgFullFilename);
  cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

  // push image into data frame buffer
  frame.cameraImg = imgGray;

  if (dataBuffer.size() < dataBufferSize){
    dataBuffer.push_back(frame);
  }
  else{
    dataBuffer.erase(dataBuffer.begin());
    dataBuffer.push_back(frame);
  }
}

//Log data of number of keypoints with regard to respective detector  
void DataLog(vector<string> detectorType_list, vector<string> descriptorType_list, int imgStartIndex, int imgEndIndex, bool data_save)
{
  if (data_save){

    // Match descriptor
    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
    string descriptorType = "DES_HOG"; // DES_BINARY, DES_HOG
    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
    
    // data location
    string dataPath = "../";
    
    // Filename path
	string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    bool bVis = false;            // visualize results
    
    // Focus on Preceding Vehicle only
  	bool bFocusOnVehicle = true;
  	cv::Rect vehicleRect(535, 180, 180, 150);
    
    // Create the table format of the output "Data_Log.csv"
  	ofstream Num_keypoints, Num_matches, Time_log;
  	Num_keypoints.open("../Result/Keypoints.csv");
    Num_matches.open("../Result/Matches.csv");
    Time_log.open("../Result/Time_Log.csv");
    
    Num_keypoints << "Detector Type" << "," << "Descroptor Type" << ",";
    Num_matches << "Detector Type" << "," << "Descroptor Type" << ",";
    Time_log << "Detector Type" << "," << "Descroptor Type" << ",";
    for (int i=imgStartIndex; i<= imgEndIndex - imgStartIndex; i++)
    {
    	Num_keypoints << "Image" << i << ",";  
     	Time_log << "Image" << i << "_" << "detect(ms)" << "," << "Image" << i << "_" << "desc(ms)" << ","; 
      	if (i == imgStartIndex){
          continue;
        }
      	else{
          Num_matches << "Image" << i-1 << "-" << i << ","; 
        }
    }
    Num_keypoints << endl;
    Num_matches << endl;
    Time_log << endl;
    
    // Loop each detector type
    for (auto it=detectorType_list.begin(); it<detectorType_list.end(); ++it)
    {
      
      // Loop each descriptor type 
      for (auto i=descriptorType_list.begin(); i<descriptorType_list.end(); ++i)
      {
        Num_keypoints << *it << "," << *i << "," ;
        Num_matches  << *it << "," << *i << "," ;
        Time_log  << *it << "," << *i << "," ;
        vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
        // Loop each image with regard to specific combination of detector and descriptor type
        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    	{
            DataFrame frame;
         	vector<cv::KeyPoint> keypoints_temp;
         	cv::Mat descriptors_temp;
          	// Load Data and save it into buffer
            LoadData(imgFillWidth, imgStartIndex, imgIndex, imgBasePath, imgPrefix, imgFileType, frame, dataBuffer, dataBufferSize);
            cv::Mat img_temp = (dataBuffer.end() - 1)->cameraImg;
          
            // Detecting keypoints
            double t_detect = Detector(*it, keypoints_temp, img_temp, bVis);
        	int num_total = keypoints_temp.size();
          	Time_log << t_detect << ","; // Log the time of detection
          
			// Focus on preceding vehicle
        	FocusOnVehicle(bFocusOnVehicle, vehicleRect, keypoints_temp);
          	int num_focus = keypoints_temp.size();
          	Num_keypoints << num_focus << "/" << num_total << ","; // Log the number of keypoints
          
          	// push keypoints and descriptor for current frame to end of data buffer
        	(dataBuffer.end() - 1)->keypoints = keypoints_temp;

            // AKAZE detector works only with AKAZE descriptor
            if ((i == descriptorType_list.begin()+4) && (it != detectorType_list.begin()+5)){
              cout << *it << " detector doesn't work with AKAZE descriptor" << endl;
              if (imgIndex > 0) // write N.A into csv
              {
              	Num_matches << "N/A" << "," ;
              }
              Time_log << "N/A" << "," ;
              continue;
            }
            // SIFT detector and ORB descriptor do not work together
            else if((i == descriptorType_list.begin()+2) && (it == detectorType_list.begin()+6)){
              cout << *it << " detector doesn't work with " << *i << "descriptor" << endl;
              if (imgIndex > 0) // write N.A into csv
              {
              	Num_matches << "N/A" << "," ;
              }
              Time_log << "N/A" << "," ;
              continue;
            }
            else{
              double t_desc = descKeypoints(keypoints_temp, img_temp, descriptors_temp, *i);
              Time_log << t_desc << ",";
            }

          	// push descriptors for current frame to end of data buffer
            (dataBuffer.end() - 1)->descriptors = descriptors_temp;
          	
          	if (dataBuffer.size() > 1) // wait until at least two images have been processed
            {
              Match(dataBuffer, matcherType, descriptorType, selectorType);
              int num_matches = (dataBuffer.end() - 1)->kptMatches.size();
              Num_matches << num_matches << "," ;
            } 
          	cout << endl;
      	}
        Num_keypoints << endl;
        Num_matches << endl;
        Time_log << endl;
        cout << endl;
      }
    }
  }
}
//// MAIN PROGRAM ////
int main(int argc, const char *argv[])
{

    //// INIT VARIABLES AND DATA STRUCTURES ////
	// detector and descriptor type
	vector<string> detectorType_list{"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
  	vector<string> descriptorType_list{"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
  	string detectorType = "SHITOMASI"; // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
  	string descriptorType = "SIFT"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
  
    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

  	// data log
  	bool data_save = true;	
  	DataLog(detectorType_list, descriptorType_list, imgStartIndex, imgEndIndex, data_save);
  
  	// Focus on Preceding Vehicle only
  	bool bFocusOnVehicle = true;
  	cv::Rect vehicleRect(535, 180, 180, 150);
  
    //// MAIN LOOP OVER ALL IMAGES ////
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
      
         //// LOAD IMAGE INTO BUFFER ////
        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
      	cout << "**" << imgFullFilename << endl;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
      
      	if (dataBuffer.size() < dataBufferSize){
        	dataBuffer.push_back(frame);
        }
      	else{
        	dataBuffer.erase(dataBuffer.begin());
          	dataBuffer.push_back(frame);
        }
        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        //// DETECT IMAGE KEYPOINTS ////

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
		double t1 = Detector(detectorType, keypoints, imgGray, bVis);
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle
        // only keep keypoints on the preceding vehicle
        FocusOnVehicle(bFocusOnVehicle, vehicleRect, keypoints);
        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;
      	cout<< "Number of keypoints identified for the preceding vehicle:" << keypoints.size() << endl;

        //// EXTRACT KEYPOINT DESCRIPTORS ////

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        cv::Mat descriptors;
        double t2 = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;
        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {
            //// MATCH KEYPOINT DESCRIPTORS ////

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType_match = "DES_HOG"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType_match, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
          	cout<< "Number of matched keypoints identified for the preceding vehicle:" << matches.size() << endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                
              	cv::rectangle(matchImg, vehicleRect, cv::Scalar(255, 0, 0),3);
              	cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
              	cout << "-------------------------------------" << endl << endl;
                //cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
          	
        }
	cout << endl;
    
    } // eof loop over all images
  
    return 0;
}


