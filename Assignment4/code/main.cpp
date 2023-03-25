#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
int point_num = 4;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < point_num) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 2)
        return t * control_points[0] + (1 - t) * control_points[1];
    else {
        std::vector<cv::Point2f> sub_control_points;
        for (int i = 0; i < control_points.size() - 1; i++) {
            sub_control_points.push_back(t * control_points[i] + (1 - t) * control_points[i + 1]);
        }
        
        return recursive_bezier(sub_control_points, t);
    }

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    
    bool antialising = true;
    if (antialising) {
        for (float t = 0; t <= 1; t += 0.001) {
            cv::Point2f point = recursive_bezier(control_points, t);

            float dx = point.x - std::floor(point.x);
            float dy = point.y - std::floor(point.y);

            //判断point的四个相邻点及其最近点
            cv::Point2f p00, p01, p10, p11, nearest;
            if (dx <= 0.5f && dy <= 0.5f) {
                p00 = cv::Point2f(std::floor(point.x) - 1, std::floor(point.y) - 1);
                p01 = cv::Point2f(std::floor(point.x), std::floor(point.y) - 1);
                p10 = cv::Point2f(std::floor(point.x) - 1, std::floor(point.y));
                p11 = cv::Point2f(std::floor(point.x), std::floor(point.y));
                nearest = p11;
            }
            else if (dx <= 0.5f && dy > 0.5f) {
                p00 = cv::Point2f(std::floor(point.x) - 1, std::floor(point.y));
                p01 = cv::Point2f(std::floor(point.x), std::floor(point.y));
                p10 = cv::Point2f(std::floor(point.x) - 1, std::floor(point.y) + 1);
                p11 = cv::Point2f(std::floor(point.x), std::floor(point.y) + 1);
                nearest = p01;
            }
            else if (dx > 0.5f && dy <= 0.5f) {
                p00 = cv::Point2f(std::floor(point.x), std::floor(point.y) - 1);
                p01 = cv::Point2f(std::floor(point.x) + 1, std::floor(point.y) - 1);
                p10 = cv::Point2f(std::floor(point.x), std::floor(point.y));
                p11 = cv::Point2f(std::floor(point.x) + 1, std::floor(point.y));
                nearest = p10;
            }
            else if (dx > 0.5f && dy > 0.5f) {
                p00 = cv::Point2f(std::floor(point.x), std::floor(point.y));
                p01 = cv::Point2f(std::floor(point.x) + 1, std::floor(point.y));
                p10 = cv::Point2f(std::floor(point.x), std::floor(point.y) + 1);
                p11 = cv::Point2f(std::floor(point.x) + 1, std::floor(point.y) + 1);
                nearest = p00;
            }

            std::vector<cv::Point2f> points{ p00,p01,p10,p11 };

            float near_dis = sqrt((point.x - nearest.x) * (point.x - nearest.x) + (point.y - nearest.y) * (point.y - nearest.y));

            for (auto p : points) {
                float dis = sqrt((p.x - nearest.x) * (p.x - nearest.x) + (p.y - nearest.y) * (p.y - nearest.y));
                float ratio = near_dis / dis;
                window.at<cv::Vec3b>(point.y, point.x)[1] = 255;    //绘制运动点
                cv::Vec3b color = window.at<cv::Vec3b>(point.y, point.x);
                window.at<cv::Vec3b>(p.y, p.x) = ratio * color;     //绘制相邻点
            }
        }
    }
    else {
        for (float t = 0; t <= 1; t += 0.001) {
            cv::Point2f point = recursive_bezier(control_points, t);
            window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == point_num) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
