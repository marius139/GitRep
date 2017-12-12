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

//Start of class
class Route
{
private:
//Counts the number of points that have been initialized
unsigned int stops_initialized;

//Vector for storing points clicked in RViz
std::vector<geometry_msgs::PointStamped> points;
//Int for determening how many points should be stored in the vector
int sizeP;

//Int for storing randomly generated number for making
int iRand;
//Vector for choosing next goal after reaching a goal
std::vector<int> iOrdVec;

    // creating a client for MoveBaseAction messages, to send messages to the movebase which is used to move the robot
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
    // creating a publisher for the markers
    ros::Publisher marker_pub;
    // creating a subscriber to the clicking
    ros::Subscriber click_sub;

    //a function to send the goal to the movebase
    //goal_point is an object of type PointStamped (data in this message comes from function _clicked_point_cb
    void _send_goal(const geometry_msgs::PointStamped& goal_point)
    {
        //a initilization of the message goal eveything comes from the _clicked_point_cb through goal_point
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = goal_point.header.frame_id;
        goal.target_pose.pose.position = goal_point.point;
        goal.target_pose.pose.orientation.w = 1.0;
        // creates a pointer when _taget_reached_cb is called. This points function that sends goal to _send_markers
        client.sendGoal(goal, boost::bind(&Route::_target_reached_cb, this, _1, _2));
        _send_markers();
    }
    //_send_goal publishes the PointStamped messages -> the base then automatically generates the velocity message to reach desired point

    void _target_reached_cb(const actionlib::SimpleClientGoalState& state,
       const move_base_msgs::MoveBaseResultConstPtr& result)
    {
      if (iOrdVec.size()==0){
        for(int i=0; i<points.size(); i++){
          iOrdVec.push_back(i);
        }
      }

      srand (time(NULL));
      iRand = rand() % iOrdVec.size();

      _send_goal(points[iOrdVec[iRand]]);
      iOrdVec.erase(iOrdVec.begin()+iRand);
    }

    //Publishes the markers to rviz
    void _send_markers()
    {
        //Design of the visualisation of the markers in rviz
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
        
        //Adds the same amount of visual markers, as the number of points in the vector
        for(int i=0; i<points.size();i++){

          if (stops_initialized > i)
          {
              marker.header.frame_id = points[i].header.frame_id;   //Sets the same frame to the virtual marker, as the point placed in rviz
              marker.id = i;                                        //ID of the marker
              marker.pose.position = points[i].point;               //Sets the position of the virtual marker to the same position as the point placed in rviz
              marker.pose.position.z += 1.0;                        //Sets the visual marker to be over the floor in rviz
              marker_array.markers.push_back(marker);               //Stores the marker in the markerarray
          }
        }
        
        //sends the data to rviz
        marker_pub.publish(marker_array);

    }

      //The following function executes when the user clicks in rviz
      //The geometry_msgs::PointStamped includes coordinates and a timestamp
    void _clicked_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        //prints the coordinates of the marker clicked to the terminal
        ROS_INFO("Clicked: %f, %f, %f", msg->point.x,
            msg->point.y, msg->point.z);
        
        //If there are more stops, that have not yet been initialized. 
        if (stops_initialized < sizeP)
        {
            points.push_back(*msg); //save the coordinates at the end of the vector, containing the waypoints.
        }
        
        //if all the points have been initialized, and the user wants to place another point.
        if (stops_initialized >= sizeP)
        {
            //Swap the points, so that the first point initialized is shifted all the way to the back.
          for(int i=0; i < points.size()-1; i++){   
              std::swap(points[i],points[i+1]);
            }
            //Replace the previous first point placed, with the new one. This will be the last point in the vector.
            points.back() = *msg;
        }
        
        //a counter that counts how many points have been placed in the vector
        stops_initialized += 1;
        
        //sends the first point as a goal for the turtlebot to move to.
        _send_goal(points[0]);
        //Clear vector for chosing next point to ensure we choose between the right number of points
        iOrdVec.clear();

    }

public:
    //class constructor
    Route() :
    //sets stops initialized to an intial value and the client to movebase
        stops_initialized(0), client("move_base")
    {
      //sets variable for amount of points we want
        sizeP = 4;

        //Ros nodehandle
        ros::NodeHandle n;
        //Advertise markers to patrolRoute_markers
        marker_pub = n.advertise<visualization_msgs::MarkerArray>(
            "patrolRoute_markers", 1);
        //Subscribe to clicked points in RViz
        click_sub = n.subscribe( "clicked_point", 100,
            &Route::_clicked_point_cb, this);
            
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
