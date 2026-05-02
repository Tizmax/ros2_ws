
#include <vector>
#include <string>
#include <map>
#include <list>


#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cs7630_msgs/msg/trajectory.hpp>
#include <cs7630_msgs/msg/trajectory_element.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;
class PathFollower : public rclcpp::Node {
    protected:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<cs7630_msgs::msg::Trajectory>::SharedPtr traj_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        double look_ahead_;
        double Kx_,Ky_,Ktheta_;
        double max_rot_speed_;
        double max_velocity_;
        double max_y_error_;
        double max_error_;
        double max_angular_error_;
        double max_tracking_delay_;
        double replan_period_;
        double period_;
        bool always_publish_;
        double tracking_delay_;
        int replan_counter_;
        geometry_msgs::msg::PoseStamped goal_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


        std::string frame_id_, base_frame_;

        typedef std::map<double, cs7630_msgs::msg::TrajectoryElement> Trajectory;

        Trajectory traj_;

        void traj_cb(cs7630_msgs::msg::Trajectory::SharedPtr msg) {
            frame_id_ = msg->header.frame_id;
            traj_.clear();
            for (unsigned int i=0;i<msg->ts.size();i++) {
                traj_.insert(Trajectory::value_type(rclcpp::Time(msg->ts[i].header.stamp).seconds(), msg->ts[i]));
            }
            RCLCPP_INFO(this->get_logger(),"Trajectory received");
        }

        void goal_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            goal_ = *msg;
            RCLCPP_INFO(this->get_logger(),"Goal cached for replanning");
        }

        geometry_msgs::msg::Pose2D computeError(const rclcpp::Time & now, const cs7630_msgs::msg::TrajectoryElement & te) {
            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                // Check that frame_id_ has been set (it comes from trajectory messages)
                if (frame_id_.empty()) {
                    RCLCPP_ERROR(this->get_logger(),"Cannot compute error: frame_id not set. Waiting for trajectory message.");
                    return geometry_msgs::msg::Pose2D();
                }
                std::string errStr;
                if (!tf_buffer->canTransform(base_frame_,frame_id_,now,rclcpp::Duration(1s),&errStr)) {
                    RCLCPP_ERROR(this->get_logger(),"Cannot transform target: %s",errStr.c_str());
                    return geometry_msgs::msg::Pose2D();
                }
                transformStamped = tf_buffer->lookupTransform(base_frame_,frame_id_, now);
            } catch (const tf2::TransformException & ex){
                RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
                return geometry_msgs::msg::Pose2D();
            }
            geometry_msgs::msg::PoseStamped pose,error;
            pose.header.stamp = now;
            pose.header.frame_id = frame_id_;
            pose.pose  = te.pose;
            tf2::doTransform(pose,error,transformStamped);
            geometry_msgs::msg::Pose2D result;
            result.x = error.pose.position.x;
            result.y = error.pose.position.y;
            result.theta = tf2::getYaw(error.pose.orientation);
            // RCLCPP_INFO(this->get_logger(),"Current error: %+6.2f %+6.2f %+6.2f",result.x,result.y,result.theta*180./M_PI);
            return result;
        }

    public:
        PathFollower(): rclcpp::Node("path_follower") {
            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/look_ahead",1.0);
            this->declare_parameter("~/Kx",1.0);
            this->declare_parameter("~/Ky",1.0);
            this->declare_parameter("~/Ktheta",1.0);
            this->declare_parameter("~/max_rot_speed",1.0);
            this->declare_parameter("~/max_velocity",1.0);
            this->declare_parameter("~/max_y_error",1.0);
            this->declare_parameter("~/max_error",0.5);
            this->declare_parameter("~/max_angular_error",M_PI/3);
            this->declare_parameter("~/max_tracking_delay",3.0);
            this->declare_parameter("~/period",0.050);
            this->declare_parameter("~/replan_period",100.0);
            this->declare_parameter("~/always_publish",true);

            base_frame_ = this->get_parameter("~/base_frame").as_string();
            look_ahead_ = this->get_parameter("~/look_ahead").as_double();
            Kx_ = this->get_parameter("~/Kx").as_double();
            Ky_ = this->get_parameter("~/Ky").as_double();
            Ktheta_ = this->get_parameter("~/Ktheta").as_double();
            max_rot_speed_ = this->get_parameter("~/max_rot_speed").as_double();
            max_velocity_ = this->get_parameter("~/max_velocity").as_double();
            max_y_error_ = this->get_parameter("~/max_y_error").as_double();
            max_error_ = this->get_parameter("~/max_error").as_double();
            max_angular_error_ = this->get_parameter("~/max_angular_error").as_double();
            max_tracking_delay_ = this->get_parameter("~/max_tracking_delay").as_double();
            period_ = this->get_parameter("~/period").as_double();
            replan_period_ = this->get_parameter("~/replan_period").as_double();
            always_publish_ = this->get_parameter("~/always_publish").as_bool();
            tracking_delay_ = 0.0;
            replan_counter_ = 0;

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            // Initialize the transform broadcaster
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


            traj_sub_ = this->create_subscription<cs7630_msgs::msg::Trajectory>("~/traj",1,
                    std::bind(&PathFollower::traj_cb,this,std::placeholders::_1));
            twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/twistCommand",1);
            pose2d_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("~/error",1);


            goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/goal",1,
                std::bind(&PathFollower::goal_cb,this,std::placeholders::_1));
            goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/goal",1);

            timer_ = this->create_wall_timer( std::chrono::duration<double>(period_),
                    std::bind(&PathFollower::timer_cb, this));


        };

    protected:

        inline double sat(double x, double max_x) {
            if (x < -max_x) return -max_x;
            if (x > +max_x) return +max_x;
            return x;
        }

        void replan() {
            geometry_msgs::msg::PoseStamped goal = goal_;
            goal.header.stamp = this->get_clock()->now();
            goal_pub_->publish(goal);
            RCLCPP_INFO(this->get_logger(),"Replan requested by republishing current goal");
        }

        void timer_cb() {
            if (traj_.size() > 0) {
                bool final = false;
                bool should_replan = false;

                rclcpp::Time now = this->get_clock()->now();
                // First find the reference point which corresponds best to
                // the current time. 
                // TODO: modify this part to react to tracking delays
                // introduced by obstacle avoidance or switch to manual.
                Trajectory::const_iterator it = traj_.lower_bound(now.seconds() - tracking_delay_ + look_ahead_);
                if (it == traj_.end()) {
                    // let's keep the final position
                    it --;
                    final = true;
                }
                // Now broadcast the reference pose as a TF for
                // visualization
                geometry_msgs::msg::TransformStamped t;
                // Read message content and assign it to
                // corresponding tf variables
                t.header.stamp = now;
                t.header.frame_id = frame_id_;
                t.child_frame_id = "carrot";

                t.transform.translation.x = it->second.pose.position.x;
                t.transform.translation.y = it->second.pose.position.y;
                t.transform.translation.z = 0.0;
                t.transform.rotation = it->second.pose.orientation;
                // Send the transformation
                tf_broadcaster_->sendTransform(t);



                // Compute the tracking error and 
                geometry_msgs::msg::Pose2D error = computeError(now,it->second);
                pose2d_pub_->publish(error);
                if (hypot(error.x,error.y)>max_error_ || abs(error.theta) > max_angular_error_) {      
                    // TODO: Manage the fact that the error has good too far.
                    // We need to make the carrot stop by delaying the requested time by 
                    // one time period (period_).
                    // After some time, we should probably trigger a replanning
                    // add the time of while to the delay
                    // Freeze/slow the carrot by accumulating effective delay.
                    tracking_delay_ = std::min(tracking_delay_ + period_, max_tracking_delay_);
                    if (tracking_delay_ >= max_tracking_delay_) {
                        should_replan = true;
                    }
                } else if (tracking_delay_ > 0.0) {
                    // Recover gradually once tracking error is back under control.
                    tracking_delay_ = std::max(0.0, tracking_delay_ - period_);
                }
                // RCLCPP_INFO(this->get_logger(),"tracking delay: %.2f sec / %.2f sec", tracking_delay_, max_tracking_delay_);

                geometry_msgs::msg::Twist twist;
                if (final && (abs(error.x) < 0.1) && (abs(error.y) < 0.1) && (abs(error.theta) < M_PI/18)) {
                    // Finished
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    // We're done
                    RCLCPP_INFO(this->get_logger(),"Final position reached, stopping with error: %.2f %.2f %.2f",error.x,error.y,error.theta);

                    traj_.clear();
                } else {
                    twist.linear.x = it->second.twist.linear.x + Kx_ * error.x;
                    twist.linear.x = std::min(twist.linear.x,max_velocity_);
                    error.y = sat(error.y,max_y_error_);
                    twist.angular.z = it->second.twist.angular.z + Ktheta_ * error.theta 
                        + Ky_*it->second.twist.linear.x*exp(-error.theta*error.theta)*error.y;
                    twist.angular.z = sat(twist.angular.z,max_rot_speed_);

                    // printf("Twist: %.2f %.2f\n",twist.linear.x,twist.angular.z);
                }
                twist_pub_->publish(twist);
                
                if (!final) {
                    replan_counter_++;
                    if (replan_counter_ > replan_period_ || should_replan) {
                        replan_counter_ = 0;
                        replan();
                    }
                }

            } else if (always_publish_) {
                // Publish zero velocity
#if 0
                geometry_msgs::msg::Twist twist;
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                twist_pub_->publish(twist);
#endif
            }

        }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
}


