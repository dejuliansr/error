#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <array>
#include <numeric>
#include <vector>

// template <typename T>
// std::vector<T> operator-(const std::vector<T>& lhs, const std::vector<T>& rhs) {
//    std::vector<T> res;
//    res.reserve(lhs.size());
   
//    for (size_t idx = 0; idx < lhs.size(); idx++) {
//       res.push_back(lhs[idx] - rhs[idx]);
//    }
   
//    return res;
// }

class ControllerActionServer
{
protected:
    ros::NodeHandle nh;
    ros::Publisher jointPub;
    ros::Subscriber jointSub;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as;
    std::string action_name;
    control_msgs::FollowJointTrajectoryFeedback feedback;
    control_msgs::FollowJointTrajectoryResult result;
    sensor_msgs::JointState currentState;
    trajectory_msgs::JointTrajectory targetState;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    
public:
    static constexpr double max_position_error = 5;
    ControllerActionServer(std::string name) : as(nh, name, boost::bind(&ControllerActionServer::executeCB, this, _1), false), action_name(name)
    {
        jointPub = nh.advertise<trajectory_msgs::JointTrajectory>("dynamixel_workbench/joint_trajectory", 100);
        as.start();
    }

    ~ControllerActionServer(void)
    {
    }

    void executeCB(control_msgs::FollowJointTrajectoryGoalConstPtr const &goal)
    {
        ROS_INFO_STREAM("Starting path...");
        targetState = goal->trajectory;

        targetState.header.stamp = ros::Time::now();
        jointPub.publish(targetState);

        result.error_code = result.SUCCESSFUL;
        ROS_INFO_STREAM("Current trajectory complete");
        as.setSucceeded(result);

        std::stringstream goalTrajectoryPointPositionsString;
        std::vector<trajectory_msgs::JointTrajectoryPoint> goalTrajectoryPoints = goal->trajectory.points;
        std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator goalTrajectoryPointsIterator;
        int i_points = 0;
        for (goalTrajectoryPointsIterator = goalTrajectoryPoints.begin(); goalTrajectoryPointsIterator < goalTrajectoryPoints.end(); goalTrajectoryPointsIterator++)
        {
            std::stringstream goalTrajectoryPointPositionsStream;
            std::copy(goalTrajectoryPointsIterator->positions.begin(), goalTrajectoryPointsIterator->positions.end(), std::ostream_iterator<double>(goalTrajectoryPointPositionsStream, " "));
            ROS_INFO_STREAM(action_name.c_str() << ": Received New Planning, With Goal Points " << i_points << ": " << goalTrajectoryPointPositionsStream.str());
            i_points++;
        }
        
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_action_server");

    ControllerActionServer controllerActionServer("follow_joint_trajectory");

    ros::Rate loop_rate(250);
    loop_rate.sleep();
    ros::spin();

    return 0;
}