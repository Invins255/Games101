//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        // 坐标限定,对越界的u,v值进行处理
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        
        //(u,v)在textrue上的位置
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        
        //双线性插值
        int x0 = floor(u_img), x1 = ceil(u_img);
        int y0 = floor(v_img), y1 = ceil(v_img);
        float s = u_img - x0;
        float t = v_img - y0;
        auto color1 = image_data.at<cv::Vec3b>(y0, x0);
        auto color2 = image_data.at<cv::Vec3b>(y0, x1);
        auto color3 = image_data.at<cv::Vec3b>(y1, x0);
        auto color4 = image_data.at<cv::Vec3b>(y1, x1);

        auto color_0 = color1 + s * (color2 - color1);
        auto color_1 = color3 + s * (color4 - color3);

        auto color = color_0 + t * (color_1 - color_0);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
