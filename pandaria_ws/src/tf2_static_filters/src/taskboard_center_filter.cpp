#include <Eigen/Dense>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class Filter
{
  private:
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    geometry_msgs::TransformStamped stable_center_tf_;
    ros::Publisher pub_;
    ros::Subscriber subscriber_;
    int64_t state_;
    static constexpr size_t filter_size_ = 100;
    Eigen::Matrix<double, 3, filter_size_> translation_filter_buffer_;
    Eigen::Matrix<double, 4, filter_size_> orientation_filter_buffer_;
    size_t current_col_ = 0;
    bool filtering_;

  public:
    Filter(ros::NodeHandle *nh)
    {
        filtering_  = false;
        state_      = 0;
        subscriber_ = nh->subscribe("/startFilterCenter", 1000, &Filter::tf_Callback, this);
    }

    void tf_Callback(const std_msgs::Int64 &msg)
    {
        if (state_ != 1)
        {
            state_ = msg.data;
        }
    }

    void startFiltering(tf2_ros::Buffer &tf_buffer)
    {

        if (state_ == 1)
        {
            geometry_msgs::TransformStamped static_transformStamped;
            std::string err_string;

            if (tf_buffer.canTransform("panda_link0", "taskboard_center", ros::Time(0), ros::Duration(1), &err_string) && current_col_ < filter_size_)
            {

                auto center                                  = tf_buffer.lookupTransform("panda_link0", "taskboard_center", ros::Time(0));
                translation_filter_buffer_.col(current_col_) = tf2::transformToEigen(center).translation();
                orientation_filter_buffer_.col(current_col_) = Eigen::Vector4d(center.transform.rotation.w, center.transform.rotation.x,
                                                                               center.transform.rotation.y, center.transform.rotation.z);

                current_col_++;
                ROS_INFO_STREAM("Got " << current_col_ << "nd transform");
            }

            if (current_col_ == filter_size_)
            {
                if (!filtering_)
                {
                    // filter translation
                    const Eigen::Vector3d meanTrans              = translation_filter_buffer_.rowwise().mean();
                    const Eigen::Matrix<double, 1, -1> distTrans = (translation_filter_buffer_.colwise() - meanTrans).colwise().norm();
                    size_t minTransIndex;
                    distTrans.minCoeff(&minTransIndex);

                    const Eigen::Vector4d meanOrient              = orientation_filter_buffer_.rowwise().mean();
                    const Eigen::Matrix<double, 1, -1> distOrient = (orientation_filter_buffer_.colwise() - meanOrient).colwise().norm();
                    size_t minOrientIndex;
                    distOrient.minCoeff(&minOrientIndex);

                    stable_center_tf_.transform.translation.x = translation_filter_buffer_.col(minTransIndex).x();
                    stable_center_tf_.transform.translation.y = translation_filter_buffer_.col(minTransIndex).y();
                    stable_center_tf_.transform.translation.z = translation_filter_buffer_.col(minTransIndex).z();
                    stable_center_tf_.transform.rotation.w    = orientation_filter_buffer_.col(minOrientIndex)[0];
                    stable_center_tf_.transform.rotation.x    = orientation_filter_buffer_.col(minOrientIndex)[1];
                    stable_center_tf_.transform.rotation.y    = orientation_filter_buffer_.col(minOrientIndex)[2];
                    stable_center_tf_.transform.rotation.z    = orientation_filter_buffer_.col(minOrientIndex)[3];

                    stable_center_tf_.header.frame_id = "panda_link0";
                    stable_center_tf_.child_frame_id  = "taskboard_center_stable";
                    stable_center_tf_.header.stamp    = ros::Time::now();

                    filtering_ = true;
                }
                else
                {
                    static_broadcaster_.sendTransform(stable_center_tf_);
                }
            }
            else
                ROS_INFO_STREAM_DELAYED_THROTTLE(2, "Waiting for taskboard_center " << err_string);
        }
        else
        {
            ROS_INFO_STREAM_DELAYED_THROTTLE(2, "State: " << state_);
            current_col_ = 0;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "taskboard_center_filter");
    ros::NodeHandle nh("taskboard_center_filter");

    Filter taskboard_center = Filter(&nh);
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Rate r(33);  // 10 hz
    while (ros::ok())
    {
        taskboard_center.startFiltering(tf_buffer);
        ros::spinOnce();
        r.sleep();
    }
}
