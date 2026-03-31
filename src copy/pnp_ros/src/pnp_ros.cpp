#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "robot_interfaces/action/catch.hpp"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>  // ⭐ 新增
#include <vector>
#include <array>
#include <deque>
#include <cmath>

using namespace std;
using namespace cv;

// ================================================================
//  常量配置区
// ================================================================
static const float BOX_MM = 350.0f;
static const float HALF = BOX_MM / 2.f;
static const double MIN_AREA = 8000.0;
static const int SMOOTH_N = 6;

static const vector<Point3f> OBJ_PTS = {
    {-HALF, -HALF, 0},  // 0 左上
    { HALF, -HALF, 0},  // 1 右上
    { HALF,  HALF, 0},  // 2 右下
    {-HALF,  HALF, 0}   // 3 左下
};

// ⭐ 移除静态 K 和 D，改为从 RealSense 动态获取

// ================================================================
//  卡尔曼滤波器封装
// ================================================================
struct CornerKF {
    KalmanFilter kf;
    bool initialized = false;
    
    void init(Point2f pt) {
        kf.init(4, 2, 0, CV_64F);
        setIdentity(kf.transitionMatrix);
        kf.transitionMatrix.at<double>(0,2) = 1.0;
        kf.transitionMatrix.at<double>(1,3) = 1.0;
        
        kf.measurementMatrix = Mat::zeros(2, 4, CV_64F);
        kf.measurementMatrix.at<double>(0,0) = 1.0;
        kf.measurementMatrix.at<double>(1,1) = 1.0;
        
        setIdentity(kf.processNoiseCov, Scalar(0.05));
        setIdentity(kf.measurementNoiseCov, Scalar(0.8));
        setIdentity(kf.errorCovPost, Scalar(1));
        
        kf.statePost = (Mat_<double>(4,1) << pt.x, pt.y, 0, 0);
        initialized = true;
    }
    
    Point2f update(Point2f measured) {
        if (!initialized) init(measured);
        Mat pred = kf.predict();
        Mat meas = (Mat_<double>(2,1) << measured.x, measured.y);
        Mat est = kf.correct(meas);
        return Point2f((float)est.at<double>(0), (float)est.at<double>(1));
    }
    
    Point2f predictOnly() {
        Mat pred = kf.predict();
        return Point2f((float)pred.at<double>(0), (float)pred.at<double>(1));
    }
};

// ================================================================
//  工具函数
// ================================================================
vector<Point2f> sortCorners(const vector<Point2f>& in) {
    vector<Point2f> r(4);
    vector<float> s(4), d(4);
    for (int i = 0; i < 4; i++) { 
        s[i] = in[i].x + in[i].y; 
        d[i] = in[i].x - in[i].y; 
    }
    
    int tl=0, br=0, tr=0, bl=0;
    for (int i = 1; i < 4; i++) {
        if (s[i] < s[tl]) tl = i;
        if (s[i] > s[br]) br = i;
        if (d[i] > d[tr]) tr = i;
        if (d[i] < d[bl]) bl = i;
    }
    
    r[0]=in[tl]; r[1]=in[tr]; r[2]=in[br]; r[3]=in[bl];
    return r;
}

Vec3d euler(const Mat& R) {
    double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0) +
                     R.at<double>(1,0)*R.at<double>(1,0));
    bool sg = sy < 1e-6;
    
    double x = sg ? atan2(-R.at<double>(1,2), R.at<double>(1,1))
                  : atan2( R.at<double>(2,1), R.at<double>(2,2));
    double y = atan2(-R.at<double>(2,0), sy);
    double z = sg ? 0 : atan2(R.at<double>(1,0), R.at<double>(0,0));
    
    return Vec3d(x * 180/CV_PI, y * 180/CV_PI, z * 180/CV_PI);
}

void putLabel(Mat& img, const string& text, Point org,
              double scale=0.65, Scalar color=Scalar(0,255,255), int thick=2) {
    putText(img, text, org, FONT_HERSHEY_SIMPLEX, scale, Scalar(0,0,0), thick+2);
    putText(img, text, org, FONT_HERSHEY_SIMPLEX, scale, color, thick);
}

// ⭐ 新增：从 RealSense 内参获取相机矩阵
Mat getK(const rs2_intrinsics& intr) {
    return (Mat_<double>(3,3) <<
        intr.fx, 0,       intr.ppx,
        0,       intr.fy, intr.ppy,
        0,       0,       1);
}

Mat getD(const rs2_intrinsics& intr) {
    Mat D = Mat::zeros(1, 5, CV_64F);
    for (int i = 0; i < 5; i++)
        D.at<double>(0, i) = intr.coeffs[i];
    return D;
}

// ================================================================
//  核心：从彩色帧里提取箱子四角
// ================================================================
bool detectBoxCorners(const Mat& frame, vector<Point2f>& corners, Mat& debugMask) {
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    Mat m1, m2, mask;
    inRange(hsv, Scalar(0,  80, 60), Scalar(10,  255, 255), m1);
    inRange(hsv, Scalar(165, 80, 60), Scalar(180, 255, 255), m2);
    mask = m1 | m2;
    
    Mat k5 = getStructuringElement(MORPH_RECT, Size(5,5));
    Mat k3 = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(mask, mask, MORPH_CLOSE, k5, Point(-1,-1), 2);
    morphologyEx(mask, mask, MORPH_OPEN,  k3, Point(-1,-1), 1);
    
    debugMask = mask.clone();
    
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;
    
    int best = 0;
    double maxA = 0;
    for (int i = 0; i < (int)contours.size(); i++) {
        double a = contourArea(contours[i]);
        if (a > maxA) { maxA = a; best = i; }
    }
    if (maxA < MIN_AREA) return false;
    
    vector<Point> hull;
    convexHull(contours[best], hull);
    
    vector<Point> poly;
    double peri = arcLength(hull, true);
    for (double eps = 0.02; eps <= 0.15; eps += 0.01) {
        approxPolyDP(hull, poly, eps * peri, true);
        if ((int)poly.size() <= 5) break;
    }
    
    RotatedRect rr = minAreaRect(hull);
    Point2f rpts[4];
    rr.points(rpts);
    
    vector<Point2f> raw(4);
    if ((int)poly.size() == 4 && isContourConvex(poly)) {
        for (int i = 0; i < 4; i++) 
            raw[i] = Point2f((float)poly[i].x, (float)poly[i].y);
    } else {
        for (int i = 0; i < 4; i++) 
            raw[i] = rpts[i];
    }
    
    raw = sortCorners(raw);
    
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    
    for (auto& p : raw) {
        p.x = clamp(p.x, 0.f, (float)(frame.cols - 1));
        p.y = clamp(p.y, 0.f, (float)(frame.rows - 1));
    }
    
    TermCriteria tc(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.01);
    cornerSubPix(gray, raw, Size(7, 7), Size(-1, -1), tc);
    
    corners = raw;
    return true;
}

// ================================================================
//  距离 / 位姿平滑
// ================================================================
struct PoseSmootherVec3 {
    deque<Vec3d> buf;
    int maxN;
    
    explicit PoseSmootherVec3(int n) : maxN(n) {}
    
    Vec3d push(Vec3d v) {
        buf.push_back(v);
        if ((int)buf.size() > maxN) buf.pop_front();
        
        Vec3d sum(0,0,0);
        for (auto& x : buf) sum += x;
        return sum * (1.0 / buf.size());
    }
};

// ================================================================
//  ROS 2 节点
// ================================================================
class BoxPnPNode : public rclcpp::Node {
public:
    using Catch = robot_interfaces::action::Catch;
    using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;
    
    BoxPnPNode() : Node("box_pnp_node") {
        RCLCPP_INFO(this->get_logger(), "BoxPnPNode 启动");
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        action_client_ = rclcpp_action::create_client<Catch>(this, "robotic_task_");
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("box_pose", 10);
        
        // ⭐ 启动 RealSense 替代 USB 摄像头
        RCLCPP_INFO(this->get_logger(), "正在启动 RealSense 相机...");
        try {
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
            cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
            
            rs2::pipeline_profile prof = pipe_.start(cfg);
            
            // ⭐ 从 RealSense 获取相机内参
            auto color_stream = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            rs2_intrinsics intr = color_stream.get_intrinsics();
            
            K_ = getK(intr);
            D_ = getD(intr);
            
            cout << "K:\n" << K_ << "\nD:\n" << D_ << "\n";
            
            RCLCPP_INFO(this->get_logger(), "RealSense 相机已启动");
            
        } catch (const rs2::error& e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense 错误: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&BoxPnPNode::process_frame, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "节点初始化完成");
    }
    
    ~BoxPnPNode() {
        pipe_.stop();  // ⭐ 停止 RealSense
        cv::destroyAllWindows();
    }

private:
    void process_frame() {
        // ⭐ 从 RealSense 获取帧
        rs2::frameset frames;
        try {
            frames = pipe_.wait_for_frames(100);
        } catch (const rs2::error& e) {
            return;
        }
        
        rs2::video_frame cf = frames.get_color_frame();
        if (!cf) return;
        
        // ⭐ 转换为 OpenCV Mat
        Mat frame(Size(cf.get_width(), cf.get_height()),
                  CV_8UC3, (void*)cf.get_data(), Mat::AUTO_STEP);
        
        Mat show = frame.clone();  // ⭐ 直接使用，不需要去畸变
        Mat debugMask;
        vector<Point2f> rawCorners;
        
        bool detected = detectBoxCorners(frame, rawCorners, debugMask);
        
        // ----- 卡尔曼平滑角点 -----
        vector<Point2f> smoothCorners(4);
        if (detected) {
            for (int i = 0; i < 4; i++)
                smoothCorners[i] = kfs_[i].update(rawCorners[i]);
        } else {
            for (int i = 0; i < 4; i++) {
                if (kfs_[i].initialized)
                    smoothCorners[i] = kfs_[i].predictOnly();
                else
                    smoothCorners[i] = Point2f(0,0);
            }
        }
        
        bool anyInited = kfs_[0].initialized;
        
        if (anyInited) {
            // ----- 画角点框 -----
            for (int i = 0; i < 4; i++) {
                Point2f a = smoothCorners[i];
                Point2f b = smoothCorners[(i+1)%4];
                line(show, a, b, Scalar(0, 220, 0), 3, LINE_AA);
                circle(show, a, 7, Scalar(0, 0, 255), -1, LINE_AA);
                putLabel(show, to_string(i), a + Point2f(8, -8), 0.65, Scalar(255, 255, 0));
            }
            
            // ----- solvePnP -----
            Mat rvec, tvec;
            bool pnp_ok = solvePnP(OBJ_PTS, smoothCorners, K_, D_,  // ⭐ 使用动态 K_ 和 D_
                                   rvec, tvec, false, SOLVEPNP_ITERATIVE);
            
            if (pnp_ok) {
                Vec3d tv(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                Vec3d tvSmooth = tvecSmoother_.push(tv);
                
                double X = tvSmooth[0];
                double Y = tvSmooth[1];
                double Z = tvSmooth[2];
                double dist = sqrt(X*X + Y*Y + Z*Z);
                
                Mat R;
                Rodrigues(rvec, R);
                Vec3d eu = euler(R);
                
                Mat tvecSmoothed = (Mat_<double>(3,1) << X, Y, Z);
                
                vector<Point3f> axisPts = {{0,0,0}, {100,0,0}, {0,100,0}, {0,0,100}};
                vector<Point2f> axisImg;
                projectPoints(axisPts, rvec, tvecSmoothed, K_, D_, axisImg);
                
                arrowedLine(show, axisImg[0], axisImg[1], Scalar(0,0,255), 3, LINE_AA, 0, 0.2);
                arrowedLine(show, axisImg[0], axisImg[2], Scalar(0,255,0), 3, LINE_AA, 0, 0.2);
                arrowedLine(show, axisImg[0], axisImg[3], Scalar(255,0,0), 3, LINE_AA, 0, 0.2);
                
                putLabel(show, "X", axisImg[1] + Point2f(5, 0), 0.6, Scalar(0,0,255));
                putLabel(show, "Y", axisImg[2] + Point2f(5, 0), 0.6, Scalar(0,255,0));
                putLabel(show, "Z", axisImg[3] + Point2f(5, 0), 0.6, Scalar(255,100,0));
                
                vector<Point2f> centerImg;
                projectPoints(vector<Point3f>{{0,0,0}}, rvec, tvecSmoothed, K_, D_, centerImg);
                if (!centerImg.empty()) {
                    int cx = (int)centerImg[0].x;
                    int cy = (int)centerImg[0].y;
                    line(show, Point(cx-14, cy), Point(cx+14, cy), Scalar(0,255,255), 2, LINE_AA);
                    line(show, Point(cx, cy-14), Point(cx, cy+14), Scalar(0,255,255), 2, LINE_AA);
                    circle(show, centerImg[0], 5, Scalar(0,255,255), -1, LINE_AA);
                }
                
                {
                    Mat overlay = show.clone();
                    rectangle(overlay, Point(10, 10), Point(500, 185), Scalar(0,0,0), FILLED);
                    addWeighted(overlay, 0.45, show, 0.55, 0, show);
                    
                    int bx = 20, by = 35, dy = 30;
                    char buf[256];
                    
                    sprintf(buf, "Distance : %.1f mm", dist);
                    putLabel(show, buf, Point(bx, by), 0.78, Scalar(0, 255, 100), 2);
                    
                    sprintf(buf, "X=%.1f  Y=%.1f  Z=%.1f  (mm)", X, Y, Z);
                    putLabel(show, buf, Point(bx, by + dy), 0.62, Scalar(0, 220, 255));
                    
                    sprintf(buf, "Roll=%.1f  Pitch=%.1f  Yaw=%.1f  (deg)", eu[0], eu[1], eu[2]);
                    putLabel(show, buf, Point(bx, by + dy*2), 0.62, Scalar(255, 200, 0));
                    
                    string status = detected ? "[ DETECT: OK ]" : "[ DETECT: LOST - KF predict ]";
                    Scalar scol = detected ? Scalar(0,255,0) : Scalar(0,100,255);
                    putLabel(show, status, Point(bx, by + dy*3), 0.62, scol);
                    
                    sprintf(buf, "Corners(px): (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f)",
                            smoothCorners[0].x, smoothCorners[0].y,
                            smoothCorners[1].x, smoothCorners[1].y,
                            smoothCorners[2].x, smoothCorners[2].y,
                            smoothCorners[3].x, smoothCorners[3].y);
                    putLabel(show, buf, Point(bx, by + dy*4), 0.52, Scalar(180, 180, 180));
                    
                    printf("\rDist=%.1fmm  XYZ=[%.1f, %.1f, %.1f]mm  RPY=[%.1f, %.1f, %.1f]deg   ",
                           dist, X, Y, Z, eu[0], eu[1], eu[2]);
                    fflush(stdout);
                }
                
                // ⭐ 构造 camera_link 坐标系下的位姿
                geometry_msgs::msg::PoseStamped pose_camera;
                pose_camera.header.stamp = this->now();
                pose_camera.header.frame_id = "camera_link";
                pose_camera.pose.position.x = tvSmooth[0] / 1000.0;
                pose_camera.pose.position.y = tvSmooth[1] / 1000.0;
                pose_camera.pose.position.z = tvSmooth[2] / 1000.0;
                
                tf2::Matrix3x3 tf2_rot(
                    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
                );
                tf2::Quaternion q;
                tf2_rot.getRotation(q);
                
                pose_camera.pose.orientation.x = q.x();
                pose_camera.pose.orientation.y = q.y();
                pose_camera.pose.orientation.z = q.z();
                pose_camera.pose.orientation.w = q.w();
                
                // ⭐ 尝试TF转换到 base_link
                try {
                    geometry_msgs::msg::PoseStamped pose_base = 
                        tf_buffer_->transform(pose_camera, "base_link", tf2::durationFromSec(0.1));
                    
                    last_valid_pose_ = pose_base;
                    has_valid_pose_ = true;
                    
                    pose_pub_->publish(pose_base);
                    
                    if (!goal_sent_) {
                        goal_sent_ = true;
                        send_catch_goal(pose_base.pose);
                    }
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "TF转换失败: %s", ex.what());
                }
            }
        }
        
        // ⭐ 如果当前没检测到，但之前有有效位姿，就发布上一次的
        if (!anyInited || !detected) {
            if (has_valid_pose_) {
                last_valid_pose_.header.stamp = this->now();
                pose_pub_->publish(last_valid_pose_);
            }
        }
        
        // ---- 显示 mask 在右下角 ----
        {
            Mat maskBGR, maskSmall;
            cvtColor(debugMask, maskBGR, COLOR_GRAY2BGR);
            resize(maskBGR, maskSmall, Size(320, 180));
            
            int ox = show.cols - 325;
            int oy = show.rows - 185;
            maskSmall.copyTo(show(Rect(ox, oy, maskSmall.cols, maskSmall.rows)));
            putLabel(show, "RedMask", Point(ox+5, oy+18), 0.55, Scalar(200,200,200));
        }
        
        imshow("Box PnP", show);
        waitKey(1);
    }
    
    void send_catch_goal(const geometry_msgs::msg::Pose& target_pose) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Action Server 未连接");
            return;
        }
        
        auto goal_msg = Catch::Goal();
        goal_msg.target_pose = target_pose;
        goal_msg.action_type = 2;
        
        RCLCPP_INFO(this->get_logger(), "发送抓取目标");
        
        auto send_goal_options = rclcpp_action::Client<Catch>::SendGoalOptions();
        send_goal_options.result_callback = 
            std::bind(&BoxPnPNode::result_cb, this, std::placeholders::_1);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    void result_cb(const GoalHandleCatch::WrappedResult &result) {
        RCLCPP_INFO(this->get_logger(), "抓取任务完成: %s", result.result->reason.c_str());
    }
    
    // ⭐ 成员变量：用 RealSense pipeline 替代 VideoCapture
    rs2::pipeline pipe_;
    Mat K_, D_;  // ⭐ 动态获取，不再是静态常量
    
    array<CornerKF, 4> kfs_;
    PoseSmootherVec3 tvecSmoother_{SMOOTH_N};
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp_action::Client<Catch>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    bool goal_sent_ = false;
    geometry_msgs::msg::PoseStamped last_valid_pose_;
    bool has_valid_pose_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxPnPNode>());
    rclcpp::shutdown();
    return 0;
}
