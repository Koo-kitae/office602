/*
    1 = Recharging Area (Default)
    2 = Sickroom
    3 = Exhausting Area
*/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

using namespace std;

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    // Connect to ROS
    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up so that we can begin processing goals.
    while(!ac.waitForServer(ros::Duration(5.0))){
    // ROS_INFO("move_base action 서버 대기 중");
    cout << "move_base action 서버 대기 중" << endl;
    }

    int user_choice = 1;
    char choice_to_continue = 'Y';
    bool run = true;

    while(run) {

    // Ask the user where he wants the robot to go?
    cout << "\n목표 지점을 입력하세요" << endl;
    cout << "\n1 = 충전 구역" << endl;
    cout << "2 = 병실 1" << endl;
    cout << "3 = 폐기 구역" << endl;
    cout << "\n숫자를 입력하세요: ";
    cin >> user_choice;

    // Create a new goal to send to move_base 
    move_base_msgs::MoveBaseGoal goal;

    // Send a goal to the robot
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    bool valid_selection = true;

    // Use map_server to load the map of the environment on the /map topic. 
    // Launch RViz and click the Publish Point button in RViz to 
    // display the coordinates to the /clicked_point topic.
    switch (user_choice) {
        case 1:
        cout << "\n목표 지점 : 충전 구역\n" << endl;
        goal.target_pose.pose.position.x = -0.217960834503;
        goal.target_pose.pose.position.y = -1.04363012314;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.678093828428;
        goal.target_pose.pose.orientation.w = 0.734975346422;
        break;
        case 2:
        cout << "\n목표 지점 : 병실\n" << endl;
        goal.target_pose.pose.position.x = 2.5;
        goal.target_pose.pose.position.y = 0.15;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z =  0.027398240287736;  // -0.0364083684981
        goal.target_pose.pose.orientation.w = 0.999624597751144;
        break;
        case 3:
        cout << "\n목표 지점 : 폐기 구역\n" << endl;
        goal.target_pose.pose.position.x = 0.314909696579;
        goal.target_pose.pose.position.y = 1.12254846096;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.41090931658603;
        goal.target_pose.pose.orientation.w = 0.999155411002427;
        break;
        default:
        cout << "\n유효하지 않은 값입니다. 다시 입력해주세요.\n" << endl;
        valid_selection = false;
    }       

    // Go back to beginning if the selection is invalid.
    if(!valid_selection) {
        continue;
    }

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait until the robot reaches the goal
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        // ROS_INFO("로봇이 목표 지점에 도달했습니다");
        cout << "로봇이 목표 지점에 도달했습니다" << endl;
    else
        // ROS_INFO("로봇이 목표지점에 도달하지 못했습니다");
        cout << "로봇이 목표지점에 도달하지 못했습니다" << endl;

    // Ask the user if he wants to continue giving goals
    do {
        cout << "\n계속 진행하시겠습니까? (Y/N)" << endl;
        cin >> choice_to_continue;
      choice_to_continue = tolower(choice_to_continue); // Put your letter to its lower case
        } 
    while (choice_to_continue != 'n' && choice_to_continue != 'y'); 

    if(choice_to_continue =='n') {
        run = false;
        }  
    }

    return 0;
}
