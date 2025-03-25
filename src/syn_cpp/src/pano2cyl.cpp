#include <opencv2/opencv.hpp>
#include <cmath>

#define DTOR (M_PI / 180)
#define EPS 0.0001

static void CalcLongLat(double x, double y, double &longitude, double &latitude, double lat1, double lat2, double vfov, double long1, double long2) {
    static bool first = true;
    static double y0, tanmaxlat, tanlat1, tanlat2;

    if (first) {
        y0 = (tan(lat1) + tan(lat2)) / (tan(lat1) - tan(lat2));  // middle latitude
        tanmaxlat = tan(0.5 * vfov);
        tanlat1 = tan(lat1) / (-1 - y0);
        tanlat2 = tan(lat2) / (1 - y0);
        first = false;
    }

    longitude = long1 + x * (long2 - long1);  // -pi to pi
    if (fabs(fabs(lat1) - fabs(lat2)) < EPS) {
        latitude = atan(y * tanmaxlat);
    } else {
        if (y > y0)
            latitude = atan((y - y0) * tanlat2);
        else
            latitude = atan((y - y0) * tanlat1);
    }
}

static cv::Mat SphereToCylinder(const cv::Mat &sphere_image, int cyl_width, double vfov, double lat1, double lat2, double long1, double long2) {
    int spherewidth = sphere_image.cols;
    int sphereheight = sphere_image.rows;
    int cyl_height = cyl_width * tan(0.5 * vfov) / (0.5 * (long2 - long1));
    cv::Mat cyl_image(cyl_height, cyl_width, sphere_image.type());

    for (int j = 0; j < cyl_height; j++) {
        for (int i = 0; i < cyl_width; i++) {
            double x = static_cast<double>(i) / (cyl_width - 1);
            double y = static_cast<double>(j) / (cyl_height - 1);
            double longitude, latitude;
            CalcLongLat(x, y, longitude, latitude, lat1, lat2, vfov, long1, long2);
            int sphere_x = static_cast<int>((longitude + M_PI) / (2 * M_PI) * (spherewidth - 1));
            int sphere_y = static_cast<int>((latitude + M_PI_2) / M_PI * (sphereheight - 1));
            cyl_image.at<cv::Vec3b>(j, i) = sphere_image.at<cv::Vec3b>(sphere_y, sphere_x);
        }
    }

    return cyl_image;
}
