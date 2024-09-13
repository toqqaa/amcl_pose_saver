# Pose Database Saver

## Overview

The `PoseDatabaseSaver` ROS node subscribes to the `amcl_pose` topic to receive and save robot pose data into an SQLite database. This includes the robot's position, orientation, and covariance matrix, which are logged periodically.

## Features

* **Database Management** : Opens and manages an SQLite database to store pose data.
* **Pose Subscription** : Subscribe to the `amcl_pose` topic for incoming robot pose messages.
* **Periodic Saving** : Uses a timer to regularly save pose data into the database.

## Setup

1. **Database Location** :

* The SQLite database is stored at  `/data_base/pose_database.db`.

1. **Topic Subscription** :

* Subscribes to the `amcl_pose` topic to retrieve pose data.

1. **Timer** :

* Saves pose data to the database every second.

## Building and Running

```bash
catkin_make
rosrun pose_saver pose_database_saver 
```


## Viewing Data

To view the stored data, you can use an SQLite viewer extension or tool of your choice. This will allow you to inspect and query the pose data saved in the database.
