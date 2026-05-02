
#include <vector>
#include <string>
#include <map>
#include <list>


#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cs7630_msgs/msg/trajectory.hpp>
#include <cs7630_msgs/msg/trajectory_element.hpp>
#include <cmath>
#include <algorithm>


using std::placeholders::_1;
using namespace std::chrono_literals;

class PathOptimizer : public rclcpp::Node {
    protected:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Publisher<cs7630_msgs::msg::Trajectory>::SharedPtr traj_pub_;
        double velocity_;
        double max_acceleration_;
        double max_braking_;
        double max_rot_speed_;

        void path_cb(nav_msgs::msg::Path::SharedPtr msg) {
            // First pre-compute heading and curvilinear abscissa.
            const double EPS_DIST = 1e-6;
            std::vector<double> s(msg->poses.size(),0.0);
            std::vector<double> heading(msg->poses.size(),0.0);
            std::vector<double> ds_vec(msg->poses.size(),0.0);
            for (unsigned i=1;i<msg->poses.size();i++) {
                const geometry_msgs::msg::Point& P0 = msg->poses[i-1].pose.position;
                const geometry_msgs::msg::Point& P1 = msg->poses[i].pose.position;
                double ds = hypot(P1.y-P0.y,P1.x-P0.x);
                ds_vec[i] = ds;
                if (ds <= EPS_DIST) {
                    // rotation on spot: take yaw from orientation field
                    heading[i] = tf2::getYaw(msg->poses[i].pose.orientation);
                } else {
                    heading[i] = atan2(P1.y-P0.y,P1.x-P0.x);
                }
                s[i] = s[i-1] + ds;
            }
            if (heading.size()>1) {
                heading[0] = heading[1];
            }

            // Initialize target speeds: default to desired cruise speed for motion segments,
            // and zero for pure-rotation (same x,y) segments.
            unsigned N = msg->poses.size();
            std::vector<double> v(N, velocity_);
            for (unsigned i=1;i<N;i++) {
                if (ds_vec[i] <= EPS_DIST) {
                    // mark rotation on spot: no linear motion across this step
                    v[i] = 0.0;
                    v[i-1] = 0.0;
                }
            }

            // enforce start and end at zero speed
            if (N>0) v[0] = 0.0;
            if (N>1) v[N-1] = 0.0;

            // Forward pass (acceleration limit)
            for (unsigned i=1;i<N;i++) {
                double ds = ds_vec[i];
                if (ds <= EPS_DIST) continue; // rotations handled separately
                double vmax_from_prev = std::sqrt(v[i-1]*v[i-1] + 2.0*max_acceleration_*ds);
                if (v[i] > vmax_from_prev) v[i] = vmax_from_prev;
            }

            // Backward pass (braking limit)
            for (int i=(int)N-2;i>=0;i--) {
                double ds = ds_vec[i+1];
                if (ds <= EPS_DIST) continue;
                double vmax_from_next = std::sqrt(v[i+1]*v[i+1] + 2.0*max_braking_*ds);
                if (v[i] > vmax_from_next) v[i] = vmax_from_next;
            }

            // Now compute timestamps and angular velocities (omega) between points.
            cs7630_msgs::msg::Trajectory output;
            output.header = msg->header;
            output.ts.resize(N);
            rclcpp::Time base_time = rclcpp::Time(output.header.stamp);
            std::vector<rclcpp::Time> t(N);
            t[0] = base_time;
            for (unsigned i=1;i<N;i++) {
                double ds = ds_vec[i];
                double dt = 0.0;
                if (ds <= EPS_DIST) {
                    // rotation on the spot: compute angular delta and allocate time based on max_rot_speed_
                    double dtheta = remainder(heading[i]-heading[i-1], 2*M_PI);
                    double rot_time = std::abs(dtheta) / std::max(1e-6, max_rot_speed_);
                    dt = rot_time;
                } else {
                    // trapezoidal average speed for the segment
                    double v0 = v[i-1];
                    double v1 = v[i];
                    if (v0 + v1 > 1e-9) dt = 2.0 * ds / (v0 + v1);
                    else dt = ds / std::max(1e-6, velocity_);
                }
                t[i] = t[i-1] + rclcpp::Duration(std::chrono::duration<double>(dt));
            }

            // fill output trajectory elements (timestamps, poses, twists)
            for (unsigned i=0;i<N;i++) {
                output.ts[i].header = msg->poses[i].header;
                output.ts[i].header.stamp = t[i];
                output.ts[i].pose.position = msg->poses[i].pose.position;
                tf2::Quaternion q;
                q.setRPY(0,0,heading[i]);
                output.ts[i].pose.orientation = tf2::toMsg(q);
                output.ts[i].twist.linear.x = v[i];
                output.ts[i].twist.angular.z = 0.0; // will set below for segments
            }

            // compute angular velocities per segment and assign to the target element (i)
            for (unsigned i=1;i<N;i++) {
                double dt = (t[i] - t[i-1]).seconds();
                double dtheta = remainder(heading[i]-heading[i-1], 2*M_PI);
                double omega = 0.0;
                if (dt > 1e-9) {
                    omega = dtheta / dt;
                } else {
                    omega = 0.0;
                }
                // assign angular velocity to the later sample (i) to represent motion arriving there
                output.ts[i].twist.angular.z = omega;
            }
            if (output.ts.size()>0) {
                // ensure last point zero
                unsigned int j = output.ts.size() - 1;
                output.ts[j].twist.linear.x = 0.0;
                output.ts[j].twist.angular.z = 0.0;
            }
            traj_pub_->publish(output);
            RCLCPP_INFO(this->get_logger(),"Optimized path into a trajectory");
        }

    public:
        PathOptimizer() : rclcpp::Node("path_optimizer") {
            this->declare_parameter("~/velocity",1.0);
            this->declare_parameter("~/max_acceleration",1.0);
            this->declare_parameter("~/max_braking",1.0);
            this->declare_parameter("~/max_rot_speed",1.0);
            velocity_ = this->get_parameter("~/velocity").as_double();
            max_acceleration_ = this->get_parameter("~/max_acceleration").as_double();
            max_braking_ = this->get_parameter("~/max_braking").as_double();
            max_rot_speed_ = this->get_parameter("~/max_rot_speed").as_double();
            path_sub_ = this->create_subscription<nav_msgs::msg::Path>("~/path",1,
                    std::bind(&PathOptimizer::path_cb,this,std::placeholders::_1));
            traj_pub_ = this->create_publisher<cs7630_msgs::msg::Trajectory>("~/trajectory",1);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathOptimizer>());
    rclcpp::shutdown();
}



