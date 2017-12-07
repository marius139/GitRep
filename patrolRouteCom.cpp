#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <time.h>


class Route
{
private:
//Int for counting the amount of stops we have recieved
unsigned int stops_initialized;

//Vector for storing the
std::vector<geometry_msgs::PointStamped> points;
int sizeP;


int iRand;
std::vector<int> iOrdVec;


    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
    ros::Publisher marker_pub;
    ros::Subscriber click_sub;

    void _send_goal(const geometry_msgs::PointStamped& goal_point)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = goal_point.header.frame_id;
        goal.target_pose.pose.position = goal_point.point;
        goal.target_pose.pose.orientation.w = 1.0;
        client.sendGoal(goal, boost::bind(&Route::_target_reached_cb, this, _1, _2));
        _send_markers();
    }

    void _target_reached_cb(const actionlib::SimpleClientGoalState& state,
       const move_base_msgs::MoveBaseResultConstPtr& result)
    {
      if (iOrdVec.size()==0){
        for(int i=0; i<sizeP; i++){
          iOrdVec.push_back(i);
        }
      }

      srand (time(NULL));
      iRand = rand() % iOrdVec.size() + 0;

      _send_goal(points[iOrdVec[iRand]]);
      iOrdVec.erase(iOrdVec.begin()+iRand);
    }

    void _send_markers()
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.ns = "bus_stops";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0.7071;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0.7071;
        marker.lifetime = ros::Duration();


        visualization_msgs::MarkerArray marker_array;

        for(int i=0; i<points.size();i++){

          if (stops_initialized > i)
          {
              marker.header.frame_id = points[i].header.frame_id;
              marker.id = i;
              marker.pose.position = points[i].point;
              marker.pose.position.z += 1.0;
              marker_array.markers.push_back(marker);
          }
        }
        marker_pub.publish(marker_array);

    }

      //function for clicking on a point in the map.
      //The geometry_msgs::PointStamped includes coordinates and a timestamp
    void _clicked_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        ROS_INFO("Clicked: %f, %f, %f", msg->point.x,
            msg->point.y, msg->point.z);
        if (stops_initialized < sizeP)
        {
            points.push_back(*msg);

        }
        if (stops_initialized >= sizeP)
        {
          for(int i=0; i < points.size()-1; i++){
              std::swap(points[i],points[i+1]);
            }
            points.back() = *msg;
        }
        stops_initialized += 1;
        _send_goal(points[0]);
    }

public:
    Route() :
        stops_initialized(0), client("move_base")
    {

        sizeP = 4;
        ros::NodeHandle n;

        marker_pub = n.advertise<visualization_msgs::MarkerArray>(
            "busroute_markers", 1);

        click_sub = n.subscribe( "clicked_point", 100,
            &Route::_clicked_point_cb, this);

        for(int i=0; i<sizeP; i++){
          iOrdVec.push_back(i);
        }
    };
    ~Route(){};
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "patrolRoute");
    Route r;
    ros::spin();

    return 0;
}
