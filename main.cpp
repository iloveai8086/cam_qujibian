//#include <opencv2/opencv.hpp>
//#include <chrono>
//using namespace std;
//
//int main(int argc, char **argv)
//{
//    // 内参
//    double fx = 1010.1193051331736, fy =  1009.4943272753831, cx = 1007.7481675024154, cy = 577.4709247205907;
//    /**内参矩阵K
//     * fx  0  cx
//     * 0  fy  cy
//     * 0   0   1
//     */
//    // 畸变参数
//    double k1 = -0.06663067168381479, k2 = 0.0009026610617662017, p1 = -0.007498635027107796, p2 = 0.0019139336144852457;
//
//    // cv::Mat image = cv::imread(argv[1], 0); // 图像是灰度图，CV_8UC1
//    cv::Mat image = cv::imread("/media/ros/A666B94D66B91F4D/ros/test_port/camera/front/1658972540336259470.jpg",1); // 图像是灰度图，CV_8UC1
//    int rows = image.rows, cols = image.cols;
//    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC3); // 方法1去畸变以后的图
//    cv::Mat image_undistort2 = cv::Mat(rows, cols, CV_8UC3);  // 方法2 OpenCV去畸变以后的图
//
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//    //! 方法1. 自己写计算去畸变后图像的内容
//    for (int v = 0; v < rows; v++)
//    {
//        for (int u = 0; u < cols; u++)
//        {
//            double x = (u - cx) / fx, y = (v - cy) / fy; //要求解的真实图，归一化平面上的坐标
//            double r = sqrt(x * x + y * y);
//            double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x); //畸变后归一化坐标
//            double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
//            double u_distorted = fx * x_distorted + cx; //畸变后像素坐标，即原图
//            double v_distorted = fy * y_distorted + cy;
//            // 投影赋值
//            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) //真实图畸变后仍然在图上的
//            {
//                image_undistort.at<uchar>(v, u) = image.at<uchar>((int)v_distorted, (int)u_distorted);
//            }
//            else
//            {
//                image_undistort.at<uchar>(v, u) = 0; //这里最好用插值法
//            }
//        }
//    }
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//    cout << "time = " << time_used.count() << endl;
//
//    //! 方法2. OpenCV自带的undistort函数，更快速
//    cv::Mat K = cv::Mat::eye(3, 3, CV_32FC1); //内参矩阵
//    K.at<float>(0, 0) = fx;
//    K.at<float>(1, 1) = fy;
//    K.at<float>(0, 2) = cx;
//    K.at<float>(1, 2) = cy;
//    cv::Mat distort_coeffs = cv::Mat::zeros(1, 5, CV_32FC1); //畸变系数矩阵 顺序是[k1, k2, p1, p2, k3]
//    distort_coeffs.at<float>(0, 0) = k1;
//    distort_coeffs.at<float>(0, 1) = k2;
//    distort_coeffs.at<float>(0, 2) = p1;
//    distort_coeffs.at<float>(0, 3) = p2;
//    cout << "K = " << endl
//         << K << endl;
//    cout << "distort_coeffs = " << endl
//         << distort_coeffs << endl;
//
//    t1 = chrono::steady_clock::now();
//    cv::undistort(image, image_undistort2, K, distort_coeffs); //去畸变
//    t2 = chrono::steady_clock::now();
//    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//    cout << "time = " << time_used.count() << endl;
//
//    // 展示去畸变后图像
//    cv::imshow("distorted", image);
//    cv::imshow("undistorted", image_undistort);
//    cv::imshow("image_undistort2", image_undistort2);
//    cv::waitKey(0);
//    return 0;
//}



//
// Created by jiang on 2020/4/29.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <string.h>

using namespace std;

void GetFileNames(string path, vector<string> &filenames) {
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(path.c_str())))
        return;
    while ((ptr = readdir(pDir)) != 0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            // filenames.push_back(path + "/" + ptr->d_name);
            filenames.push_back(ptr->d_name);
    }
    closedir(pDir);
}

/*
 * 修改车，那么K、D、K2都得跟着改
 * 1号车前视相机内参
```yaml
cam0:
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs:
  - -0.0526858350541784
  - -0.01873269061565343
  - 0.0060846931831152
  - -0.0016727061237763216
  distortion_model: equidistant
  intrinsics:
  - 1003.9989013289942
  - 1004.1132782586517
  - 926.3763250309561
  - 546.1004237610695
  resolution:
  - 1920
  - 1080
  rostopic: /camera6/image_raw

  3号车前视相机内参
cam0:
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs:
  - -0.06663067168381479
  - 0.0009026610617662017
  - -0.007498635027107796
  - 0.0019139336144852457
  distortion_model: equidistant
  intrinsics:
  - 1010.1193051331736
  - 1009.4943272753831
  - 1007.7481675024154
  - 577.4709247205907
  resolution:
  - 1920
  - 1080
  rostopic: /camera6/image_raw/compressed


 * */

int main() {
    //f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{_1}\f$.
//    const cv::Mat K = (cv::Mat_<double>(3, 3)
//            << 1010.1193051331736, 0.0, 1007.7481675024154, 0.0, 1009.4943272753831, 577.4709247205907, 0.0, 0.0, 1.0);
//    const cv::Mat D = (cv::Mat_<double>(4, 1)
//            << -0.06663067168381479, 0.0009026610617662017, -0.007498635027107796, 0.0019139336144852457);

    const cv::Mat K = (cv::Mat_<double>(3, 3)
            << 1003.9989013289942, 0.0, 926.3763250309561, 0.0, 1004.1132782586517, 546.1004237610695, 0.0, 0.0, 1.0);

    const cv::Mat D = (cv::Mat_<double>(4, 1)
            << -0.0526858350541784, -0.01873269061565343, 0.0060846931831152, -0.0016727061237763216);


    const string str = "/media/ros/A666B94D66B91F4D/ros/test_port/camera/front/";
    // const string old_path = "/media/ros/A666B94D66B91F4D/ros/test_port/camera/jibian/";  // 老的路径
    const string old_path = "/media/ros/A666B94D66B91F4D/ros/test_port/camera/camera_lane_test/";  // 老的路径
    const int nImage = 915;
    const int ImgWidth = 1920;
    const int ImgHeight = 1080;
    cv::Size image_size;
    image_size.width = 1920;
    image_size.height = 1080;

    cv::Mat map1, map2;
    cv::Size imageSize(ImgWidth, ImgHeight);
    const double alpha = 0;  // 控制黑边的记得
//    const cv::Mat K2 = (cv::Mat_<double>(3, 3)
//            << 1010.1193051331736 / 1.4, 0.0, 1007.7481675024154, 0.0, 1009.4943272753831 /
//                                                                       1.0, 577.4709247205907, 0.0, 0.0, 1.0);
    const cv::Mat K2 = (cv::Mat_<double>(3, 3)
            << 1003.9989013289942 / 1.6, 0.0, 926.3763250309561, 0.0, 1004.1132782586517 / 1.0, 546.1004237610695, 0.0, 0.0, 1.0);
    // 这几个参数有点小忘记了到底除以多少，黑边怎么办，弯曲怎么办？

    cv::Mat NewCameraMatrix = cv::getOptimalNewCameraMatrix(K2, D, imageSize, alpha, imageSize, 0);

    // 内参改了，但畸变系数不用改？
//    cv::Matx33d KK;
//    KK(0, 0) = 599.1767578125;
//    KK(0, 1) = 0;
//    KK(0, 2) = 1051.742061275407;
//    KK(1, 0) = 0;
//    KK(1, 1) = 989.3424682617188;
//    KK(1, 2) = 571.4513472305662;
//    KK(2, 0) = 0;
//    KK(2, 1) = 0;
//    KK(2, 2) = 1;
    std::cout<<NewCameraMatrix<<std::endl;
    // std::cout<<NewCameraMatrix.inv()<<std::endl;
    // 需要搞清楚畸变系数

    // string new_path = "/media/ros/A666B94D66B91F4D/ros/test_port/camera/qujibian2/";  // 新的路径
    string new_path = "/media/ros/A666B94D66B91F4D/ros/test_port/camera/camera_lane_test_qujibian/";  // 新的路径
    vector<string> file_name;
    GetFileNames(old_path, file_name);
    // 最好sort一下
    std::sort(file_name.begin(),file_name.end());
    for (int i = 0; i < file_name.size(); i++) {
        cout << old_path + file_name[i] << endl;
        // string InputPath = "/media/ros/A666B94D66B91F4D/ros/test_port/camera/front/1658972540336259470.jpg";
        cv::Mat RawImage = cv::imread(old_path + file_name[i]);
        //cv::imshow("RawImage", RawImage);

        cv::Mat UndistortImage;
        cv::fisheye::undistortImage(RawImage, UndistortImage, K, D, NewCameraMatrix, imageSize);
        // 这个矩阵就是两个K
        //        cv::undistort(RawImage, UndistortImage, K, D, NewCameraMatrix);
        //cv::imshow("UndistortImage", UndistortImage);

        //string OutputPath = str + to_string(i) + "_un2" + ".png";
        cv::imwrite(new_path + file_name[i], UndistortImage);
        cv::waitKey(0);
    }

//    for(int i=0; i<nImage; i++)
//    {
//        string InputPath = "/media/ros/A666B94D66B91F4D/ros/test_port/camera/front/1658972540336259470.jpg";
//        cv::Mat RawImage = cv::imread(InputPath);
//        cv::imshow("RawImage", RawImage);
//
//        cv::Mat UndistortImage;
//        cv::fisheye::undistortImage(RawImage, UndistortImage, K, D, K,imageSize);
//        // 这个矩阵就是两个K
//    //        cv::undistort(RawImage, UndistortImage, K, D, NewCameraMatrix);
//        cv::imshow("UndistortImage", UndistortImage);
//
//        //string OutputPath = str + to_string(i) + "_un2" + ".png";
//        //cv::imwrite(OutputPath, UndistortImage);
//        cv::waitKey(0);
//    }

    return 0;
}

