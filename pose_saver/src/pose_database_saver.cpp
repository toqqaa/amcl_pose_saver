#include <pose_saver/robot_pose_saver.h>

PoseDatabaseSaver::PoseDatabaseSaver(ros::NodeHandle &nh)
{
    if (sqlite3_open("/home/toqa/arche_ws/src/pose_saver/data_base/pose_database.db", &db_) != SQLITE_OK) {
        ROS_ERROR("Failed to open database: %s", sqlite3_errmsg(db_));
        return;
    }
    ROS_INFO("Database opened successfully.");

    createTable();

    pose_sub_ = nh.subscribe("amcl_pose", 10, &PoseDatabaseSaver::poseCallback, this);
    timer_ = nh.createTimer(ros::Duration(1.0), &PoseDatabaseSaver::savePoseToDB, this);
}

PoseDatabaseSaver::~PoseDatabaseSaver()
{
    sqlite3_close(db_);
}

void PoseDatabaseSaver::createTable()
{
    const char* create_table_sql = "CREATE TABLE IF NOT EXISTS pose_data ("
                                    "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                                    "timestamp INTEGER, "
                                    "x REAL, "
                                    "y REAL, "
                                    "z REAL, "
                                    "qx REAL, "
                                    "qy REAL, "
                                    "qz REAL, "
                                    "qw REAL, "
                                    "cov00 REAL, "
                                    "cov01 REAL, "
                                    "cov02 REAL, "
                                    "cov03 REAL, "
                                    "cov04 REAL, "
                                    "cov05 REAL, "
                                    "cov10 REAL, "
                                    "cov11 REAL, "
                                    "cov12 REAL, "
                                    "cov13 REAL, "
                                    "cov14 REAL, "
                                    "cov15 REAL, "
                                    "cov20 REAL, "
                                    "cov21 REAL, "
                                    "cov22 REAL, "
                                    "cov23 REAL, "
                                    "cov24 REAL, "
                                    "cov25 REAL, "
                                    "cov30 REAL, "
                                    "cov31 REAL, "
                                    "cov32 REAL, "
                                    "cov33 REAL, "
                                    "cov34 REAL, "
                                    "cov35 REAL, "
                                    "cov36 REAL);";
    char* errMsg;
    if (sqlite3_exec(db_, create_table_sql, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("Failed to create table: %s", errMsg);
        sqlite3_free(errMsg);
    } else {
        ROS_INFO("Table created successfully.");
    }
}

void PoseDatabaseSaver::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
   
    current_pose_ = msg->pose.pose;
    for (size_t i = 0; i < 36; ++i)
    {
        covariance_[i] = msg->pose.covariance[i];
    }
}

void PoseDatabaseSaver::savePoseToDB(const ros::TimerEvent&)
{
    
    sqlite3_stmt* stmt;
    const char* insert_sql = "INSERT INTO pose_data (timestamp, x, y, z, qx, qy, qz, qw, "
                             "cov00, cov01, cov02, cov03, cov04, cov05, "
                             "cov10, cov11, cov12, cov13, cov14, cov15, "
                             "cov20, cov21, cov22, cov23, cov24, cov25, "
                             "cov30, cov31, cov32, cov33, cov34, cov35, cov36) "
                             "VALUES (?, ?, ?, ?, ?, ?, ?, ?, "
                             "?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
    
    if (sqlite3_prepare_v2(db_, insert_sql, -1, &stmt, 0) != SQLITE_OK) {
        ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db_));
        return;
    }

    // Timestamp (
    int timestamp = ros::Time::now().toSec();
    sqlite3_bind_int(stmt, 1, timestamp);

    // Position
    sqlite3_bind_double(stmt, 2, current_pose_.position.x);
    sqlite3_bind_double(stmt, 3, current_pose_.position.y);
    sqlite3_bind_double(stmt, 4, current_pose_.position.z);

    // Orientation
    sqlite3_bind_double(stmt, 5, current_pose_.orientation.x);
    sqlite3_bind_double(stmt, 6, current_pose_.orientation.y);
    sqlite3_bind_double(stmt, 7, current_pose_.orientation.z);
    sqlite3_bind_double(stmt, 8, current_pose_.orientation.w);

    
    for (size_t i = 0; i < 37; ++i)
    {
        sqlite3_bind_double(stmt, 9 + i, covariance_[i]);
    }

    if (sqlite3_step(stmt) != SQLITE_DONE) {
        ROS_ERROR("Failed to execute statement: %s", sqlite3_errmsg(db_));
    }

    sqlite3_finalize(stmt);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_database_saver");
    ros::NodeHandle nh;

    PoseDatabaseSaver poseSaver(nh);  

    ros::spin();

    return 0;
}