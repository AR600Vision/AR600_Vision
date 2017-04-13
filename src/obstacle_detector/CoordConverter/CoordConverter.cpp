//
// Created by user on 13.04.17.
//

#include "CoordConverter.h"

tf::Vector3 CoordConverter::ToVector3(float x, float y)
{
    return tf::Vector3(x, y, 0);
}

tf::Vector3 CoordConverter::ToVector3(Vector2f v)
{
    return tf::Vector3(v(0), v(1), 0);
}

tf::Vector3 CoordConverter::ToVector3(Vector2i v)
{
    return tf::Vector3(v(0), v(1), 0);
}

geometry_msgs::Pose CoordConverter::ToPose(tf::Vector3 v)
{
    geometry_msgs::Pose pose;
    pose.position.x = v.x();
    pose.position.y = v.y();
    pose.position.z = v.z();
}

tf::Vector3 CoordConverter::CoordToIndex(tf::Vector3 coord, nav_msgs::OccupancyGrid map)
{
    float resolution = map.info.resolution;
    geometry_msgs::Pose origin = map.info.origin;


    return tf::Vector3((coord.x() - origin.position.x)/resolution,
                       (coord.y() - origin.position.y)/resolution,
                       0);
}

tf::Vector3 CoordConverter::IndexToCoord(tf::Vector3 index, nav_msgs::OccupancyGrid map)
{
    float resolution = map.info.resolution;
    geometry_msgs::Pose origin = map.info.origin;

    return tf::Vector3(index.x()*resolution + origin.position.x,
                       index.y()*resolution + origin.position.y,
                       0);
}
