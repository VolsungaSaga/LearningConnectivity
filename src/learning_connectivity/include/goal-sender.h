#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <cstdlib>
#include <math>

//Reads points from csv file into a vector of goals.
void read_trajectory(std::string);

//Helper function to construct a goal message object, given x and y coordinates.
move_base_msgs::MoveBaseGoal makeGoalMessage(double x, double y);
void PIDController(std::vector<double> des_xy_coords, std::vector<double> parameters,
                    std::vector<double> u );
                    
//Allows user to specify a custom trajectory on the command line, if no file is supplied. 
void getCustomTrajectory();

//Given a current goal and the next goal, this function is intended to help calculate the angle towards the next goal in the goal list.
double calculateAngleToNextGoal(move_base_msgs::MoveBaseGoal next_goal, move_base_msgs::MoveBaseGoal current_goal);
