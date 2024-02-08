#include "Triangle.hpp"
#include "rasterizer.hpp"
#include "eigen3/Eigen/Eigen"
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

enum Axis {
    X,
    Y,
    Z
};

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

//根据轴枚举类 决定旋转方向
Eigen::Matrix4f get_model_matrix(float rotation_angle, Axis dir) {

    float angle = (rotation_angle / 180) * MY_PI;
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    switch (dir)
    {
    case X:
        model << 1,0,0,0, 
        0,std::cos(angle), -std::sin(angle),0, 
        0,std::sin(angle),std::cos(angle),0,
        0,0,0,1;
        return model;

    case Y:
        model << std::cos(angle), 0, std::sin(angle),0,
        0,1,0,0, 
        -std::sin(angle),0,std::cos(angle),0,
        0,0,0,1;
        return model;
    default:
        model << std::cos(angle), -std::sin(angle),0,0, 
        std::sin(angle),std::cos(angle),0,0, 
        0,0,1,0, 
        0,0,0,1;
        return model;
    }

}


//绕Z轴旋转
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float angle = (rotation_angle / 180) * MY_PI;

    model << std::cos(angle), -std::sin(angle),0,0, 
        std::sin(angle),std::cos(angle),0,0, 
        0,0,1,0, 
        0,0,0,1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    //ratio = r/t
    //tan(fov/2) = t/|zNear|
    //b = -t, l = -r
    //这里理论上是取绝对值，因为相机定义是从原点看向Z轴的负数方向，所以Znear和Zfar应该都为负数的。
    //但是由于zNear和zFar传进来是正数
    //如果按照下面的定义，则会是一个倒三角形
    // float t = std::tan(((eye_fov / 180) * MY_PI) / 2) * abs(zNear);
    // float r = aspect_ratio * t;
    // float b = -t;
    // float l = -r;
    // float n = zNear;
    // float f = zFar;
    //正确的应该如下：
    //按照标准坐标系应该为负数
    float n = zNear >= 0 ? -zNear : zNear;
    float f = zFar >= 0 ? -zFar : zFar;
    float t = std::tan(((eye_fov / 180) * MY_PI) / 2) * abs(n);
    float r = aspect_ratio * t;
    float b = -t;
    float l = -r;
    //直接套公式得出
    // projection << (2 * n) / (r - l) , 0, (l+r)/(l-r), 0, 
    // 0,(2 * n) / (t - b), (b+t)/(b-t), 0, 
    // 0,0, (n+f)/(n-f), -(2 * n * f) / (n - f), 
    // 0,0,1,0;

    /*
    透视投影的矩阵构建步骤：
    1. 先将视锥挤压为立方体
    2. 将立方体平移，中心点和原点重叠
    3. 将立方体缩放，将其规范到[-1,1]之间
    */
    //1. 挤压矩阵
    Eigen::Matrix4f squeeze;
    squeeze << n,0,0,0,
        0,n,0,0,
        0,0,n+f,-n*f,
        0,0,1,0;
    //2. 立方体平移矩阵
    Eigen::Matrix4f translate;
    translate << 1,0,0,-(r+l)/2,
        0,1,0,-(t+b)/2,
        0,0,1,-(n+f)/2,
        0,0,0,1;
    //3. 立方体缩放
    Eigen::Matrix4f scale;
    scale << 2/(r-l),0,0,0,
        0,2/(t-b),0,0,
        0,0,2/(n-f),0,
        0,0,0,1;
    projection = scale * translate * squeeze;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        //zNear和zFar 应该是负数，但是传入的是正数
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, Z));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
