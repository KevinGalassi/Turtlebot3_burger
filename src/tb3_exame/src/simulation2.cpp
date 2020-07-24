/********************
 * 
 * **********************************************************************
 * System for autonomous map an area with a turtlebot3 burger           *
 * The system will ran autonoumly avoiding the obstacles.               *
 * A routin is called to start follo the first wall encounter,          *
 * The bot will remain alligned to the wall and then it starts again    *
 * a random walk                                                        *
 *                                                                      *
 * Authors: Galassi Kevin, Venturi Massimo                              *
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * ************************************/


# include "turtlebot3_drive.h"


#define RANDOM_WALK_NO 500
#define FOLLOW_WALL_NO 100000
#define UPDATE_NO 4


    static uint32_t random_walk_ite = 0 ;                //    Counter of cycle in collision avoidance mode
    static uint32_t follow_wall_ite = 0;                //    COunter of cycle in follow wall mode
    int FollowWalls_flag = 0;                           //    1: TB3 is following the wall      
    static uint8_t tb3_update_no = 0;                   //    Number of times the update state is been called
    static uint8_t counter_update = 0;                  //    Choose the direction to follow  
    static uint8_t update_tb3_direction = 0;            //    used to set the new direction

    static uint8_t turn_randomizer = 1;                 //

    double scan_data_0 = 10000;                             //     initialized high to avoid problem
    int evasive = 0;                                    //     used to performe evasive manouver  
    int counter_evasive = 0;                 //     counter of cycle before to call the eva.manu when trying to reach a wall


Turtlebot3Drive::Turtlebot3Drive()
: nh_priv_("~")
{
    //Init gazebo ros turtlebot3 node
    ROS_INFO("TurtleBot3 Simulation Node Init");
    ROS_ASSERT(init());
}
Turtlebot3Drive::~Turtlebot3Drive()
{
    updatecommandVelocity(0.0, 0.0);
    ros::shutdown();
}
/********************************************************************** Init function
**********************************************************************/
bool Turtlebot3Drive::init()
{
    // initialize ROS parameter
    // std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel", "");
    // initialize variables

    escape_range_ = 30.0 * DEG2RAD;
    escape_range_2 = 90.0 * DEG2RAD;

    check_forward_dist_ = 0.7;    
    check_side_dist_ = 0.6;        
    tb3_pose_ = 0.0;
    prev_tb3_pose_ = 0.0;
    FollowWalls_flag = 0;

    random_walk_ite = 0;                
    follow_wall_ite = 0;                      
    tb3_update_no = 0;                  
    counter_update = 0;                  
    update_tb3_direction = 0;            

    turn_randomizer = 1;                 

    scan_data_0 = 10000;             
    counter_evasive = 0;                

    // initialize publishers
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    // initialize subscribers
    laser_scan_sub_ = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
    odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

    evasive = 0;
    return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    tb3_pose_ = atan2(siny, cosy);
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    uint16_t scan_angle[5] = {0, 30, 330, 90, 270};
    uint16_t scan_angle_walls[4] = {0, 300, 270, 240}; //angles used for following walls
    for (int num = 0; num < 5; num++)
    {
        if (std::isinf(msg->ranges.at(scan_angle[num])))
        {
            scan_data_[num] = msg->range_max;
        }
        else
        {
            scan_data_[num] = msg->ranges.at(scan_angle[num]);
        }
    }
    //scan data in case of following walls
    if (FollowWalls_flag) 
    {
        for (int num = 0; num < 4; num++)
        {
            if (std::isinf(msg->ranges.at(scan_angle_walls[num])))
            {
                scan_data_walls[num] = msg->range_max;
            }
            else
            {
                scan_data_walls[num] = msg->ranges.at(scan_angle_walls[num]);
            }
        }
    }
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;
    cmd_vel_pub_.publish(cmd_vel);
}

bool Turtlebot3Drive::controlLoop()
{
    static uint8_t turtlebot3_state_num = 0;
    switch(turtlebot3_state_num)
    {
// robot get direction during collision avoidance
        case GET_TB3_DIRECTION:
        // ROS_INFO(" get direction");
        random_walk_ite++;

        if (random_walk_ite < RANDOM_WALK_NO)
        {
            if (scan_data_[CENTER] > check_forward_dist_)
            {
                if (scan_data_[LEFT] < check_side_dist_)
                {
                    prev_tb3_pose_ = tb3_pose_;
                    turtlebot3_state_num = TB3_RIGHT_TURN;
                }
                else if (scan_data_[RIGHT] < check_side_dist_)
                {
                prev_tb3_pose_ = tb3_pose_;
                turtlebot3_state_num = TB3_LEFT_TURN;
                }
                else
                {
                    turtlebot3_state_num = TB3_DRIVE_FORWARD;
                }
            }
            if (scan_data_[CENTER] < check_forward_dist_)
            {
                prev_tb3_pose_ = tb3_pose_;
                turtlebot3_state_num = TB3_RIGHT_TURN;

            }
        } 
        else
        {
            random_walk_ite = 0;
            ROS_INFO("TB3_UPDATE_EXECUTION");
            turtlebot3_state_num = TB3_UPDATE_EXECUTION;
        }
        break;

// tb3 moves forward
        case TB3_DRIVE_FORWARD:
        // ROS_INFO("drive forward ");
        updatecommandVelocity(LINEAR_VELOCITY, 0.0);
        if (FollowWalls_flag == 0)
            turtlebot3_state_num = GET_TB3_DIRECTION;
        else 
            turtlebot3_state_num = TB3_FOLLOW_WALL;

        break;


// tb3 soft right turn 
        case TB3_RIGHT_TURN:
        if (FollowWalls_flag == 0)
        {
            if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
                turtlebot3_state_num = GET_TB3_DIRECTION;
            else
                updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
        }
        else 
        {
            if (fabs(prev_tb3_pose_ - tb3_pose_) >= (escape_range_/3))
                turtlebot3_state_num = TB3_FOLLOW_WALL;
            else
                updatecommandVelocity(0.2*LINEAR_VELOCITY, -0.5*ANGULAR_VELOCITY);
        }
        break;

// tb3 soft left turn 
        case TB3_LEFT_TURN:
            if (FollowWalls_flag == 0)
            {
                if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
                    turtlebot3_state_num = GET_TB3_DIRECTION;
                else
                    updatecommandVelocity(0.0, ANGULAR_VELOCITY);
            }
            else
            {
                if (fabs(prev_tb3_pose_ - tb3_pose_) >= (escape_range_/3))
                { 
                    turtlebot3_state_num = TB3_FOLLOW_WALL;
                }
                else
                    updatecommandVelocity(0.2*LINEAR_VELOCITY, 0.5*ANGULAR_VELOCITY);
            }
        break;

// tb3 hard right turn 
        case TB3_RIGHT_TURN2:
            if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_2)
                turtlebot3_state_num = GET_TB3_DIRECTION;
            else
                updatecommandVelocity(0.0, -2 * ANGULAR_VELOCITY);
        break;



// tb3 hard left turn 
        case TB3_LEFT_TURN2:
        //  ROS_INFO("left_turn2 ");
            if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_2)
                turtlebot3_state_num = GET_TB3_DIRECTION;
            else
                updatecommandVelocity(0.0, 2*ANGULAR_VELOCITY);
        break;

// update tb3 states and switch beetwen collison avoidance and wall following
        case TB3_UPDATE_EXECUTION:
            if (tb3_update_no < UPDATE_NO)
            {
                ROS_INFO("%d", tb3_update_no );
                for (counter_update = 0; counter_update < 5 ;counter_update++)
                { 
                    if ((scan_data_[counter_update] >= 3.5) && (scan_data_[update_tb3_direction]!=3.5))
                    {
                        update_tb3_direction = counter_update;
                    }
                    ROS_INFO("%f",scan_data_[counter_update]);
                }

                if (fabs(scan_data_0 - scan_data_[0]) < 0.2 && scan_data_[0] != 3.5 )
                    update_tb3_direction = 5;

                scan_data_0 = scan_data_[0];

                switch (update_tb3_direction)
                { 
                    case 0:
                        turtlebot3_state_num = TB3_DRIVE_FORWARD;
                    break;
                    case 2:
                        prev_tb3_pose_ = tb3_pose_;
                        turtlebot3_state_num = TB3_RIGHT_TURN;
                    break;
                    case 1:
                        prev_tb3_pose_ = tb3_pose_;
                        turtlebot3_state_num = TB3_LEFT_TURN;
                    break;
                    case 4:
                        prev_tb3_pose_ = tb3_pose_;
                        turtlebot3_state_num = TB3_RIGHT_TURN2;
                    break;
                    case 3:
                        prev_tb3_pose_ = tb3_pose_;
                        turtlebot3_state_num = TB3_LEFT_TURN2;
                    break;
                    case 5:
                        prev_tb3_pose_ = tb3_pose_;
                        counter_evasive = 0;
                        turtlebot3_state_num = TB3_BACKUP;                       
                    break;
                    default:
                        turtlebot3_state_num = GET_TB3_DIRECTION;
                    break;
                }
                tb3_update_no++;
                update_tb3_direction = 0;
            }
            else 
            {
                tb3_update_no = 0;
                update_tb3_direction = 0;
                FollowWalls_flag = 1;
                turtlebot3_state_num = TB3_REACH_WALL;
                ROS_INFO("Initiate follow wall");
            }
        break;

// Try to reach the wall
        case TB3_REACH_WALL:
        
        updatecommandVelocity(LINEAR_VELOCITY, 0.0);
        if (scan_data_[0]<=0.5)
        {
            updatecommandVelocity(0.0, 0.5*ANGULAR_VELOCITY);
        }

        if ((fabs(scan_data_walls[1]-scan_data_walls[3]) > 0.1) && (scan_data_walls[2] <= 1))
        {
            updatecommandVelocity(0.0, 0.0);
            ROS_INFO("I'm closed to the wall, start following it%d", FollowWalls_flag);
            turtlebot3_state_num = TB3_FOLLOW_WALL;
            counter_evasive = 0;
        }

        counter_evasive = counter_evasive + 1;
        if (counter_evasive > 2000)
        {
            counter_evasive = 0;
            ROS_INFO("Not able to reach a wall");
            turtlebot3_state_num = TB3_BACKUP;
        }
        break;
        

// Adjust steering to continue to follow the wall
        case TB3_FOLLOW_WALL:
        follow_wall_ite++;
        ROS_INFO("%d", follow_wall_ite);
        if (follow_wall_ite < FOLLOW_WALL_NO)
        {
            if (scan_data_walls[CENTER] > check_forward_dist_)
            {
                if (scan_data_walls[2] < 0.4)
                {
                    prev_tb3_pose_ = tb3_pose_;
                    turtlebot3_state_num = TB3_LEFT_TURN;
                    ROS_INFO("left adjustment");
                }
                else if (scan_data_walls[2] > 0.6)
                {
                    prev_tb3_pose_ = tb3_pose_;
                    turtlebot3_state_num = TB3_RIGHT_TURN;
                    ROS_INFO("right adjustment");
                }
                else
                {
                    turtlebot3_state_num = TB3_DRIVE_FORWARD;
                    ROS_INFO("going forward");
                }
            }
            if (scan_data_walls[CENTER] < check_forward_dist_)
            {
                prev_tb3_pose_ = tb3_pose_;
                turtlebot3_state_num = TB3_LEFT_TURN;
                ROS_INFO("close to the wall");
            }
        } 
        else
        {
            FollowWalls_flag = 0;
            follow_wall_ite = 0;
            turtlebot3_state_num = GET_TB3_DIRECTION;
            ROS_INFO("Stop to follow the wall");
        }
        break;

// Backup routine 
        case TB3_BACKUP:

        ROS_INFO("evade obstacle ");

        evasive ++;
        if (evasive < 200)
            updatecommandVelocity( -LINEAR_VELOCITY, 0.0);
        else if (evasive < 300)
            updatecommandVelocity( 0, ANGULAR_VELOCITY);
            else if (evasive > 400)
            {
                if (FollowWalls_flag=1)
                {
                    evasive = 0;
                    turtlebot3_state_num = TB3_REACH_WALL;
                }
                else
                {
                    turtlebot3_state_num = GET_TB3_DIRECTION;
                    evasive = 0;
                }
            }
        break;

// Default Case
        default:
            turtlebot3_state_num = GET_TB3_DIRECTION;
        break;
        }
    return true;
}

void Turtlebot3Drive::End_Operation()
{
    std::cout << "Stop requested, velocity equal to 0 \n";
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub_.publish(cmd_vel);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_Autonomous_drive");
  Turtlebot3Drive turtlebot3_drive;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  turtlebot3_drive.End_Operation();

  return 0;
}