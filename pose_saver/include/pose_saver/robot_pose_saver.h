#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sqlite3.h>
#include <boost/array.hpp>


class PoseDatabaseSaver
{
public:
    PoseDatabaseSaver(ros::NodeHandle &nh);
    ~PoseDatabaseSaver();

private:
    void createTable();
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void savePoseToDB(const ros::TimerEvent&);

    ros::Subscriber pose_sub_;
    ros::Timer timer_;
    geometry_msgs::Pose current_pose_;
    boost::array<double, 36> covariance_; 
    sqlite3 *db_;
        std::string db_path_; 
};
