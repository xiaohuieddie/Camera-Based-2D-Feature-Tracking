# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Rubric Points

### MP.1 Data Buffer Optimization
Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end. Please refer to code line 101 - 106.
```
if (dataBuffer.size() < dataBufferSize){
	dataBuffer.push_back(frame); // Insert new frame into buffer when buffer size is pre-defined size.
}
else{
	dataBuffer.erase(dataBuffer.begin()); // delete the leftmost item in the buffer
	dataBuffer.push_back(frame); // Insert new frame into buffer
}
```

### MP.2 Keypoint Detection
Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly. Please refer to code lines 23-40.
```
//setting detector type
string detectorType = "SHITOMASI"; // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
```
By setting a string **detectorType** to select different detector types.

### MP.3 Keypoint Removal
Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.
```
cv::Rect  vehicleRect(535, 180, 180, 150); // Pre-defined rectangle
vector<cv::KeyPoint> keypoints_cropped;
int x_min = vehicleRect.x;
int x_max = vehicleRect.x + vehicleRect.width;
int y_min = vehicleRect.y;
int y_max = vehicleRect.y + vehicleRect.height;

//Erase the keypoints which are out of the defined rectangular box
for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
{
	if ((it->pt.x < x_max) && (it->pt.x > x_min) && (it->pt.y < y_max) && (it->pt.y > y_min)){
		keypoints_cropped.push_back(*it);
	}
	else{
		continue;
	}
}
```
The code above is removing the keypoints outside of the predefined rectangle. This means to focus on the preceding vehicle only.

### MP.4 Keypoint Descriptors
Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.
```
string descriptorType = "SIFT"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
```
By setting a string **descriptorType** to select different descriptor types.

### MP.5 Descriptor Matching
Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.
```
//// MATCH KEYPOINT DESCRIPTORS ////
string matcherType = "MAT_BF"; // MAT_BF, MAT_FLANN
```
By setting a string **matcherType** to select different matcher types.

### MP.6 Descriptor Distance Ratio
Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.
```
else if (selectorType.compare("SEL_KNN") == 0)
    {  // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        double t = (double)cv::getTickCount();
        // finds the 2 best matches
        matcher->knnMatch(descSource, descRef, knn_matches, 2); 
        //count the timer
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

        // implement k-nearest-neighbor matching
        // filter matches using descriptor distance ratio test
      	double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
	 }	
   ```
### MP.7 Performance Evaluation 1
Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.
|Detector Type|Image0  |Image1  |Image2  |Image3  |Image4  |Image5  |Image6  |Image7  |Image8  |Image9  |
|-------------|--------|--------|--------|--------|--------|--------|--------|--------|--------|--------|
|SHITOMASI    |122/1370|117/1301|122/1361|117/1358|115/1333|112/1284|112/1322|120/1366|111/1389|108/1339|
|HARRIS       |17/115  |14/98   |18/113  |20/121  |25/160  |39/383  |17/85   |28/210  |25/171  |31/281  |
|FAST         |141/1824|143/1832|140/1810|149/1817|139/1793|139/1796|153/1788|142/1695|131/1749|135/1770|
|BRISK        |254/2757|274/2777|276/2741|275/2735|293/2757|275/2695|289/2715|268/2628|258/2639|249/2672|
|ORB          |87/500  |101/500 |105/500 |110/500 |106/500 |121/500 |128/500 |120/500 |117/500 |117/500 |
|AKAZE        |162/1351|157/1327|159/1311|154/1351|162/1360|163/1347|173/1363|175/1331|175/1357|175/1331|
|SIFT         |137/1438|131/1371|121/1380|135/1335|134/1305|139/1370|136/1396|147/1382|156/1463|135/1422|
Table above is the generated **Keypoints.csv** file in when the function `Datalog` is turned on (`data_save = true`). The result is saved in the path: **SFND_2D_Feature_Matching/Result/Keypoints.csv**
```
// data log
bool data_save = true;
DataLog(detectorType_list, descriptorType_list, imgStartIndex, imgEndIndex, data_save);
```

### MP.8 Performance Evaluation 2
Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
|Detector Type|Descroptor Type|Image0-1|Image1-2|Image2-3|Image3-4|Image4-5|Image5-6|Image6-7|Image7-8|Image8-9|FIELD12|
|-------------|---------------|--------|--------|--------|--------|--------|--------|--------|--------|--------|-------|
|SHITOMASI    |BRISK          |83      |80      |71      |77      |74      |69      |76      |81      |69      |       |
|SHITOMASI    |BRIEF          |95      |93      |90      |87      |90      |91      |84      |90      |84      |       |
|SHITOMASI    |ORB            |85      |84      |84      |89      |87      |75      |81      |87      |87      |       |
|SHITOMASI    |FREAK          |66      |66      |62      |61      |62      |63      |59      |65      |61      |       |
|SHITOMASI    |AKAZE          |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |       |
|SHITOMASI    |SIFT           |110     |108     |102     |101     |98      |98      |93      |105     |95      |       |
|HARRIS       |BRISK          |11      |9       |10      |11      |16      |14      |12      |20      |16      |       |
|HARRIS       |BRIEF          |12      |12      |14      |16      |16      |15      |12      |19      |20      |       |
|HARRIS       |ORB            |11      |11      |14      |16      |18      |18      |12      |20      |19      |       |
|HARRIS       |FREAK          |11      |9       |13      |13      |13      |17      |10      |16      |17      |       |
|HARRIS       |AKAZE          |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |       |
|HARRIS       |SIFT           |14      |11      |16      |18      |20      |22      |12      |23      |21      |       |
|FAST         |BRISK          |74      |86      |81      |91      |69      |81      |89      |84      |90      |       |
|FAST         |BRIEF          |82      |95      |86      |99      |87      |94      |108     |101     |87      |       |
|FAST         |ORB            |89      |96      |79      |88      |82      |97      |99      |93      |97      |       |
|FAST         |FREAK          |62      |76      |61      |75      |56      |71      |81      |76      |76      |       |
|FAST         |AKAZE          |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |       |
|FAST         |SIFT           |112     |115     |104     |114     |106     |112     |118     |112     |100     |       |
|BRISK        |BRISK          |136     |138     |132     |139     |136     |154     |136     |147     |155     |       |
|BRISK        |BRIEF          |136     |160     |129     |140     |147     |153     |157     |158     |145     |       |
|BRISK        |ORB            |92      |102     |85      |94      |82      |112     |112     |114     |118     |       |
|BRISK        |FREAK          |108     |119     |114     |114     |100     |129     |135     |124     |129     |       |
|BRISK        |AKAZE          |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |       |
|BRISK        |SIFT           |177     |187     |171     |177     |168     |190     |193     |173     |181     |       |
|ORB          |BRISK          |59      |64      |64      |74      |71      |81      |83      |70      |68      |       |
|ORB          |BRIEF          |35      |37      |37      |53      |42      |57      |57      |59      |58      |       |
|ORB          |ORB            |37      |56      |49      |52      |56      |65      |69      |59      |66      |       |
|ORB          |FREAK          |34      |32      |37      |38      |32      |40      |42      |38      |41      |       |
|ORB          |AKAZE          |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |       |
|ORB          |SIFT           |63      |78      |79      |78      |79      |92      |89      |90      |88      |       |
|AKAZE        |BRISK          |123     |111     |120     |117     |114     |119     |134     |138     |125     |       |
|AKAZE        |BRIEF          |105     |116     |110     |107     |116     |128     |133     |134     |128     |       |
|AKAZE        |ORB            |100     |95      |96      |83      |90      |117     |103     |112     |117     |       |
|AKAZE        |FREAK          |100     |104     |94      |97      |97      |115     |126     |116     |113     |       |
|AKAZE        |AKAZE          |126     |127     |124     |116     |120     |132     |137     |138     |141     |       |
|AKAZE        |SIFT           |132     |134     |129     |136     |136     |147     |147     |153     |149     |       |
|SIFT         |BRISK          |56      |61      |56      |60      |55      |53      |54      |63      |73      |       |
|SIFT         |BRIEF          |62      |70      |61      |65      |52      |58      |70      |70      |84      |       |
|SIFT         |ORB            |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |       |
|SIFT         |FREAK          |58      |61      |52      |63      |51      |51      |48      |53      |65      |       |
|SIFT         |AKAZE          |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |N/A     |       |
|SIFT         |SIFT           |81      |79      |83      |92      |90      |82      |82      |100     |101     |       |
Table above is the generated **Matches.csv** file when the function `Datalog` is turned on (`data_save = true`). The result is saved in the path: **SFND_2D_Feature_Matching/Result/Matches.csv** 

The result is based on the following setting. The BF approach is used with the descriptor distance ratio set to 0.8.
```
// Match descriptor
string matcherType = "MAT_BF"; // MAT_BF, MAT_FLANN
string descriptorType = "DES_HOG"; // DES_BINARY, DES_HOG
string selectorType = "SEL_KNN"; // SEL_NN, SEL_KNN
```

Note for the **N/A** in the table:
-   AKAZE detector works only with AKAZE descriptor
-   SIFT detector and ORB descriptor do not work together

### MP.9 Performance Evaluation 3
Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

Table below is the generated from **Time_log.csv** file when the function `Datalog` is turned on (`data_save = true`). The result is saved in the path: **SFND_2D_Feature_Matching/Result/Time_log.csv** 

|Detector Type|Descroptor Type|Image0_detect(ms)|Image0_desc(ms)|Image1_detect(ms)|Image1_desc(ms)|Image2_detect(ms)|Image2_desc(ms)|Image3_detect(ms)|Image3_desc(ms)|Image4_detect(ms)|Image4_desc(ms)|Image5_detect(ms)|Image5_desc(ms)|Image6_detect(ms)|Image6_desc(ms)|Image7_detect(ms)|Image7_desc(ms)|Image8_detect(ms)|Image8_desc(ms)|Image9_detect(ms)|Image9_desc(ms)|FIELD23|
|-------------|---------------|-----------------|---------------|-----------------|---------------|-----------------|---------------|-----------------|---------------|-----------------|---------------|-----------------|---------------|-----------------|---------------|-----------------|---------------|-----------------|---------------|-----------------|---------------|-------|
|SHITOMASI    |BRISK          |19.092           |1.57267        |17.3964          |2.29396        |17.2045          |2.39722        |16.5025          |2.26804        |16.0364          |2.31117        |17.6189          |2.23609        |15.7495          |1.38551        |12.9743          |2.47152        |16.9806          |2.25794        |16.3528          |2.28267        |       |
|SHITOMASI    |BRIEF          |17.7442          |2.2225         |16.5155          |1.39366        |15.3482          |1.37034        |17.3271          |1.36827        |15.1702          |0.6234         |12.6851          |1.3647         |15.8112          |1.32288        |15.1014          |1.32037        |17.1385          |1.20617        |15.2478          |1.19269        |       |
|SHITOMASI    |ORB            |15.296           |1.51827        |15.454           |0.946398       |16.1745          |1.13065        |17.04            |0.912413       |18.0474          |0.931911       |13.3425          |0.923883       |15.7601          |0.974564       |14.6525          |1.09366        |15.8327          |0.90228        |19.1547          |0.935552       |       |
|SHITOMASI    |FREAK          |17.1153          |45.2606        |15.0646          |38.7645        |11.0922          |38.739         |12.0165          |38.6387        |11.4482          |38.708         |13.2509          |40.8119        |13.3076          |39.0726        |11.6293          |38.8888        |11.7581          |38.3743        |10.9952          |38.747         |       |
|SHITOMASI    |AKAZE          |11.4522          |N/A            |11.742           |N/A            |10.9983          |N/A            |11.1403          |N/A            |11.5615          |N/A            |11.0662          |N/A            |11.6049          |N/A            |11.1854          |N/A            |11.5202          |N/A            |11.9234          |N/A            |       |
|SHITOMASI    |SIFT           |11.2671          |19.7052        |10.9875          |14.0915        |11.1757          |14.8274        |11.0386          |14.9315        |11.6813          |14.3588        |11.4618          |15.371         |13.4469          |15.5251        |11.2029          |15.8152        |11.2135          |17.8292        |12.4201          |16.6671        |       |
|HARRIS       |BRISK          |19.1035          |0.496278       |12.1003          |0.454662       |12.5681          |0.508724       |12.7611          |0.516471       |13.6179          |0.575544       |27.0721          |0.711769       |12.1823          |0.485303       |14.7291          |0.597597       |13.5997          |0.621628       |19.4841          |0.681375       |       |
|HARRIS       |BRIEF          |13.0955          |0.271901       |12.0394          |0.269941       |12.1998          |0.281509       |12.1064          |0.267933       |16.0374          |0.289043       |28.8291          |0.336627       |12.0587          |0.259186       |13.9417          |0.287399       |12.8112          |0.334922       |18.0319          |0.320717       |       |
|HARRIS       |ORB            |11.6998          |0.796096       |11.992           |0.816592       |12.3624          |0.880635       |14.0213          |0.827879       |13.8748          |1.28179        |28.4693          |0.802452       |12.0382          |1.07936        |14.4392          |0.784585       |13.7408          |0.789284       |19.3192          |0.92305        |       |
|HARRIS       |FREAK          |12.0695          |38.138         |11.9521          |42.1253        |12.1712          |39.6778        |12.9104          |39.5012        |12.9112          |39.4354        |26.8038          |39.192         |11.9298          |41.932         |16.3075          |42.0417        |13.2216          |39.2371        |18.301           |42.1002        |       |
|HARRIS       |AKAZE          |12.1833          |N/A            |15.0399          |N/A            |12.0703          |N/A            |12.065           |N/A            |16.7478          |N/A            |28.8271          |N/A            |15.6636          |N/A            |14.7796          |N/A            |13.0373          |N/A            |22.8927          |N/A            |       |
|HARRIS       |SIFT           |14.3806          |14.3129        |14.4697          |14.7509        |12.2048          |14.4021        |11.9399          |15.5374        |13.6973          |15.2041        |30.7051          |18.7004        |13.1438          |14.1827        |15.9862          |13.8084        |13.4008          |14.069         |17.9567          |15.6778        |       |
|FAST         |BRISK          |0.983016         |2.5568         |0.982865         |1.80488        |0.96133          |1.75117        |0.932654         |2.28427        |0.998358         |1.7019         |0.923588         |1.65055        |0.929632         |1.77251        |0.957258         |1.73105        |0.94342          |1.58371        |0.985169         |1.60395        |       |
|FAST         |BRIEF          |1.0159           |0.970731       |0.95542          |0.696207       |0.943639         |0.867858       |0.987417         |0.719902       |0.947128         |0.766876       |0.942722         |0.909518       |1.00248          |0.804926       |0.926264         |0.677494       |0.994079         |1.11798        |0.939355         |0.826052       |       |
|FAST         |ORB            |0.998246         |1.025          |1.00329          |1.01833        |1.00335          |1.06093        |0.971503         |1.06355        |0.956618         |1.10604        |0.936242         |0.949982       |0.956118         |1.0596         |0.910875         |0.993213       |1.03184          |1.05786        |0.928732         |0.980364       |       |
|FAST         |FREAK          |0.927015         |43.3065        |0.947587         |39.6946        |0.986013         |40.5377        |0.933176         |40.5699        |1.42284          |41.6088        |0.97587          |44.3883        |1.01338          |43.6278        |0.944822         |39.7796        |0.943145         |40.114         |0.93383          |40.0954        |       |
|FAST         |AKAZE          |1.08714          |N/A            |0.969075         |N/A            |0.968051         |N/A            |1.04693          |N/A            |1.00347          |N/A            |0.965427         |N/A            |1.01088          |N/A            |0.967912         |N/A            |0.95504          |N/A            |0.968329         |N/A            |       |
|FAST         |SIFT           |0.953522         |23.2105        |0.943069         |19.8835        |0.982879         |21.7093        |0.992365         |26.657         |0.95746          |23.5557        |0.955807         |22.5226        |0.94655          |20.9238        |1.04473          |20.2616        |0.97106          |18.0434        |0.960573         |18.3353        |       |
|BRISK        |BRISK          |41.5171          |3.04339        |40.4543          |3.3543         |39.8745          |3.08005        |39.4348          |3.18409        |40.5991          |3.33851        |41.0676          |3.125          |40.3538          |3.383          |40.7172          |3.05653        |40.2441          |2.94387        |39.7149          |2.82263        |       |
|BRISK        |BRIEF          |39.7641          |1.03259        |39.9256          |1.06167        |39.7551          |1.63621        |39.352           |1.06241        |39.6158          |1.15912        |39.3992          |1.08917        |39.5887          |1.11189        |38.8918          |1.03922        |39.2048          |1.01216        |40.8511          |1.04641        |       |
|BRISK        |ORB            |40.615           |4.54298        |41.4077          |4.76726        |40.3055          |4.64242        |42.5447          |4.80143        |39.9501          |4.66811        |41.9643          |4.54075        |40.1928          |4.56029        |39.3217          |4.52656        |39.708           |4.54534        |39.9071          |4.63616        |       |
|BRISK        |FREAK          |41.1784          |46.9146        |40.3725          |43.8262        |39.8774          |41.1008        |39.2151          |43.319         |41.5489          |42.0837        |40.8285          |44.4126        |39.8903          |44.7287        |40.0479          |41.5232        |39.4154          |44.1897        |40.7783          |41.2549        |       |
|BRISK        |AKAZE          |41.853           |N/A            |41.3829          |N/A            |39.9062          |N/A            |41.6857          |N/A            |39.8494          |N/A            |40.9444          |N/A            |39.9549          |N/A            |40.3727          |N/A            |41.5355          |N/A            |41.9976          |N/A            |       |
|BRISK        |SIFT           |41.2477          |43.1709        |40.9024          |42.5015        |40.0289          |42.8966        |40.4876          |44.501         |41.2078          |45.0642        |40.6289          |43.7226        |40.9529          |43.9861        |41.1855          |42.7879        |39.8706          |42.8218        |40.164           |41.4844        |       |
|ORB          |BRISK          |8.23539          |1.21908        |7.48545          |1.33454        |8.13247          |1.40165        |8.23687          |1.48785        |7.51387          |1.42908        |7.58303          |1.62401        |7.67165          |1.50299        |7.96876          |1.50981        |7.90832          |1.45898        |8.39772          |1.4428         |       |
|ORB          |BRIEF          |8.03552          |0.51358        |10.3006          |1.04295        |7.45929          |0.578534       |7.85957          |0.58044        |7.53991          |0.552175       |8.90266          |0.654701       |7.37753          |0.657205       |7.31298          |0.622295       |7.52295          |1.07004        |7.71374          |0.56768        |       |
|ORB          |ORB            |8.93205          |4.77193        |7.62793          |4.60128        |8.07761          |4.78789        |7.67305          |4.74526        |7.64153          |5.29772        |7.37113          |4.53735        |7.20915          |4.6757         |7.74552          |4.7333         |8.14816          |4.53278        |9.46381          |4.75784        |       |
|ORB          |FREAK          |7.5823           |42.9395        |7.85644          |39.7845        |7.45322          |39.8284        |7.34778          |38.8723        |7.34135          |39.4792        |7.34026          |40.4416        |7.36992          |42.1979        |7.40864          |38.928         |7.52662          |39.9168        |7.69802          |42.1996        |       |
|ORB          |AKAZE          |7.66314          |N/A            |7.66853          |N/A            |7.37528          |N/A            |7.8041           |N/A            |7.8622           |N/A            |7.88124          |N/A            |7.63612          |N/A            |7.52713          |N/A            |7.54333          |N/A            |7.62749          |N/A            |       |
|ORB          |SIFT           |7.98444          |41.3153        |7.94264          |43.4162        |10.1338          |45.4965        |7.77392          |48.1426        |7.33934          |44.6886        |8.09084          |44.8697        |7.45124          |49.3852        |7.4908           |47.0848        |7.49599          |50.2802        |7.9463           |46.8987        |       |
|AKAZE        |BRISK          |80.8445          |1.94017        |71.6929          |1.83381        |74.7636          |1.97632        |82.8973          |2.07026        |74.2683          |2.35848        |72.6748          |1.9501         |76.0112          |2.03066        |80.541           |2.10872        |75.527           |2.06623        |68.9918          |1.97168        |       |
|AKAZE        |BRIEF          |81.4169          |0.856198       |77.0757          |0.855002       |75.9621          |0.91708        |76.2053          |0.858312       |69.7487          |0.844128       |75.8486          |0.921261       |68.5053          |0.975325       |68.2038          |1.00311        |79.9473          |1.02183        |73.3987          |0.943589       |       |
|AKAZE        |ORB            |82.7086          |3.03747        |77.8748          |2.99045        |79.4393          |2.98758        |83.0421          |2.9157         |76.7389          |3.06068        |82.6086          |2.95484        |73.1347          |2.99986        |67.2855          |3.27638        |70.0465          |2.97794        |71.4422          |3.172          |       |
|AKAZE        |FREAK          |78.2905          |42.0685        |74.4715          |42.7892        |68.1413          |44.1237        |75.5514          |42.1764        |76.713           |41.4318        |72.5162          |42.0752        |65.321           |42.0886        |67.9922          |39.6166        |68.7465          |40.6538        |68.6753          |39.7004        |       |
|AKAZE        |AKAZE          |84.2216          |58.9121        |70.0625          |73.8654        |85.8077          |65.3556        |84.3111          |67.9041        |69.2965          |67.4152        |73.1233          |55.4365        |77.1524          |63.5908        |73.2311          |64.3465        |77.3323          |71.7194        |81.2857          |68.6052        |       |
|AKAZE        |SIFT           |82.0023          |25.1593        |71.3856          |23.8924        |66.438           |23.5298        |70.9687          |23.6637        |71.1033          |24.0584        |67.9595          |24.2831        |81.6452          |24.6455        |77.0341          |24.5353        |73.2682          |25.4951        |81.2633          |23.595         |       |
|SIFT         |BRISK          |131.189          |1.76501        |129.791          |1.56028        |129.46           |1.48752        |127.82           |1.66585        |128.667          |1.59546        |128.497          |1.64223        |134.135          |1.63451        |132.513          |1.68632        |127.45           |1.81729        |129.222          |1.58899        |       |
|SIFT         |BRIEF          |129.334          |0.740227       |132.639          |0.763221       |131.498          |0.751533       |130.776          |0.840801       |132.869          |0.791561       |135.562          |0.825948       |130.141          |0.8725         |133.762          |0.866775       |134.004          |0.83866        |128.319          |0.784701       |       |
|SIFT         |ORB            |131.077          |N/A            |137.656          |N/A            |131.462          |N/A            |133.045          |N/A            |133.4            |N/A            |132.38           |N/A            |134.814          |N/A            |131.867          |N/A            |132.935          |N/A            |126.656          |N/A            |       |
|SIFT         |FREAK          |126.986          |39.3532        |124.941          |39.2658        |126.841          |39.931         |126.448          |41.2826        |129.679          |40.5646        |129.938          |40.4612        |130.593          |40.5063        |133.895          |39.9034        |131.78           |41.7726        |131.966          |40.34          |       |
|SIFT         |AKAZE          |131.705          |N/A            |129.372          |N/A            |129.631          |N/A            |126.588          |N/A            |128.658          |N/A            |128.005          |N/A            |132.871          |N/A            |128.632          |N/A            |132.999          |N/A            |130.167          |N/A            |       |
|SIFT         |SIFT           |129.602          |75.8349        |96.1214          |76.4455        |129.513          |74.4732        |100.758          |77.5755        |132.076          |106.766        |130.478          |116.465        |129.674          |107.641        |126.76           |78.4877        |100.333          |78.3216        |96.6589          |76.541         |       |

Table below is sorted by ratio from high to low, which includes:
-  The mean processing time for each combination
-  The mean number of matches for each combination
- The ratio (Matches/Time) for each combination

|Detector Type|Descroptor Type|Mean Time(ms)|Mean Matches|Ratio (Matches/Time)|
|-------------|---------------|-------------|------------|--------------------|
|FAST         |BRIEF          |1.80         |93.22       |51.76               |
|FAST         |ORB            |2.00         |91.11       |45.53               |
|FAST         |BRISK          |2.80         |82.78       |29.52               |
|ORB          |BRISK          |9.35         |70.44       |7.53                |
|ORB          |BRIEF          |8.69         |48.33       |5.56                |
|SHITOMASI    |BRIEF          |17.15        |89.33       |5.21                |
|SHITOMASI    |ORB            |17.10        |84.33       |4.93                |
|FAST         |SIFT           |22.48        |110.33      |4.91                |
|ORB          |ORB            |12.73        |56.56       |4.44                |
|SHITOMASI    |BRISK          |18.74        |75.56       |4.03                |
|SHITOMASI    |SIFT           |27.50        |101.11      |3.68                |
|BRISK        |BRIEF          |40.76        |147.22      |3.61                |
|BRISK        |BRISK          |43.53        |141.44      |3.25                |
|BRISK        |ORB            |45.21        |101.22      |2.24                |
|BRISK        |SIFT           |83.96        |179.67      |2.14                |
|FAST         |FREAK          |42.38        |70.44       |1.66                |
|AKAZE        |BRIEF          |75.55        |119.67      |1.58                |
|AKAZE        |BRISK          |77.85        |122.33      |1.57                |
|ORB          |SIFT           |54.12        |81.78       |1.51                |
|BRISK        |FREAK          |83.65        |119.11      |1.42                |
|AKAZE        |SIFT           |98.59        |140.33      |1.42                |
|AKAZE        |ORB            |79.47        |101.44      |1.28                |
|SHITOMASI    |FREAK          |52.37        |62.78       |1.20                |
|HARRIS       |BRIEF          |15.41        |15.11       |0.98                |
|HARRIS       |ORB            |16.09        |15.44       |0.96                |
|AKAZE        |FREAK          |113.31       |106.89      |0.94                |
|AKAZE        |AKAZE          |143.30       |129.00      |0.90                |
|HARRIS       |BRISK          |16.29        |13.22       |0.81                |
|ORB          |FREAK          |47.95        |37.11       |0.77                |
|HARRIS       |SIFT           |30.85        |17.44       |0.57                |
|SIFT         |BRIEF          |132.70       |65.78       |0.50                |
|SIFT         |BRISK          |131.52       |59.00       |0.45                |
|SIFT         |SIFT           |204.05       |87.78       |0.43                |
|SIFT         |FREAK          |169.64       |55.78       |0.33                |
|HARRIS       |FREAK          |55.20        |13.22       |0.24                |

Based on that, the top three combinations which give a high ratio is :
|Detector Type|Descroptor Type|Ratio (Matches/Time)|
|-------------|---------------|--------------------|
|FAST         |BRIEF          |51.76               |
|FAST         |ORB            |45.53               |
|FAST         |BRISK          |29.52               |
