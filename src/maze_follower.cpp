#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_arduino_msgs/Encoders.h>
#include <robot_msgs/MakeTurn.h>
#include <robot_msgs/FollowWall.h>
#include <robot_msgs/useMazeFollower.h>
#include <math.h>

#define INVALID 1000

enum {FORWARD = 0, LEFT_TURN = 1, RIGHT_TURN = 2, FOLLOW_LEFT = 3, FOLLOW_RIGHT = 4, TWO_LEFT = 5, SMALL_EDGE_TURN = 6};
  
int front_left, front_right, back_left, back_right, forward_left, forward_right, state;

class MazeController {

public:

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber distance_sub;
    ros::Subscriber encoder_sub;

    int previous_state;
    int previous_sensor_reading[2];
    bool stop;

    //Constructor
    MazeController() {
        n = ros::NodeHandle();
        distance_sub = n.subscribe("/ir_sensor_cm", 1, &MazeController::MazeCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        encoder_sub = n.subscribe("/arduino/encoders", 1, &MazeController::EncoderCallback, this);
        turn_client = n.serviceClient<robot_msgs::MakeTurn>("/make_turn");
        follow_client = n.serviceClient<robot_msgs::FollowWall>("/follow_wall");
        stop_server = n.advertiseService("/use_maze_follower",&MazeController::stopCallback, this);
        previous_state = FORWARD;
        previous_sensor_reading[0] = 0;
        previous_sensor_reading[1] = 0;
        stop=false;
    }

    //Destructor
    ~MazeController() {}

    //Callback for stopping the Robot
    bool stopCallback(robot_msgs::useMazeFollower::Request &req, robot_msgs::useMazeFollower::Response &res){
        if(req.go){
            stop=false;
            return true;
        }
        if(!req.go){
            stop=true;
            return true;
        }
    }

    //Callback for using IR sensor values
    void MazeCallback(const ras_arduino_msgs::ADConverterConstPtr &msg) {
        front_left = msg->ch1;
        front_right = msg->ch2;
        back_left = msg->ch3;
        back_right = msg->ch4;
        forward_right = msg->ch5;
        forward_left = msg->ch6;
    }

    //Callback for the encoders
    void EncoderCallback(const ras_arduino_msgs::EncodersConstPtr &msg) {
        delta_encoder_left = msg->delta_encoder2;
        delta_encoder_right = msg->delta_encoder1;
        //ROS_INFO("Delta left: %d Delta right: %d", delta_encoder_left, delta_encoder_right);
    }

    //Method to make the robot drive forward
    void forward() {

        msg.linear.x = 0.13; //TODO Try increased speed forward and turn. Try 0.15 linear x
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        twist_pub.publish(msg);
    }

    //Method for driving the robot a specific distance
    void forward(double distance) {
        ROS_INFO("Drives forward %f cm", distance);
        double wheel_radius = 5.0;

        double distance_per_wheel = 2*M_PI*wheel_radius;
        double distance_per_tick = distance_per_wheel/360;

        int ticks = nearbyint(distance/distance_per_tick);

        msg.linear.x = 0.13;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        int left_encoder = 0;
        int right_encoder = 0;

        int tresh_front = 17;

        ros::Rate loop_rate(20);
        while (abs(left_encoder) < ticks && abs(right_encoder) < ticks) {

            ros::spinOnce();

            if ((forward_left < tresh_front &&
                forward_left > 0) ||
                (forward_right < tresh_front &&
                forward_right > 0)) break;

            left_encoder += delta_encoder_left;
            right_encoder += delta_encoder_right;

            twist_pub.publish(msg);

            loop_rate.sleep();
        }
    }
    // Sends 0 for stoping
    void stopRobot(){
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        twist_pub.publish(msg);
    }

    //Sends service message through chosen client to perform desired task
    void setClientCall(int state) {
        if (state == LEFT_TURN || state == RIGHT_TURN) {
            srv_turn.request.state = state;
            srv_turn.request.degrees = 87;//85;
            if (turn_client.call(srv_turn)) {
                ROS_INFO("Succesfully called turn service");
              }
              else
              {
                ROS_ERROR("Failed to call turn service in maze_follower");
              }
        }
        else if (state == FOLLOW_LEFT || state == FOLLOW_RIGHT) {
            srv_follow.request.state = state;
            if (follow_client.call(srv_follow)) {
                ROS_INFO("Succesfully called follow wall service");
              }
              else
              {
                ROS_ERROR("Failed to call follow service in maze_follower");
              }
        }
    }

    //Outputs if the state changes
    void changeState(int s) {
	msg.linear.x = 0;
	msg.angular.z = 0;
	twist_pub.publish(msg);
        std::cout << "want to change the state " << s << std::endl;
        std::cin.ignore();
        state = s;
    }

    //Check if there is a rapid change in distance measured by ir sensors on the left side
    bool checkSensorsDistanceLeft() {
        if (abs(front_left - previous_sensor_reading[0]) > 8) {
            return false;
        }

        return true;
    }

   //Check if there is a rapid change in distance measured by ir sensors on the right side
    bool checkSensorsDistanceRight() {
        if (abs(front_right - previous_sensor_reading[0]) > 8) {
            return false;
        }

        return true;
    }

    //Checks whenever the two sensors have registered an edge of wall
    void checkSensorsTurn() {
        bool back = false;
        bool front = false;
        ros::Rate loop_rate(10);

        while (!back || !front) {
            ros::spinOnce();
            //if (front_left < 25 && back_left < 25) break; //Necessary?
            if (wallInFront()) {
                if (front == true && back == false) {
                    setClientCall(RIGHT_TURN);
                }
                break;
            } else {
                if (front_left < 20) front = true; //TODO Try these new values for wall edge detection. Old value was 15
                if (back_left < 20) back = true;
                ROS_INFO("front: %d back: %d", front, back);
                forward();
            }
            loop_rate.sleep();
        }
    }

    void checkBackSensorTurn(int prev_state) {

        int back_sensor, front_sensor;
	ros::spinOnce();
        if (prev_state == RIGHT_TURN) {
	    ROS_INFO("Want to detect wall with back left sensor");
            previous_sensor_reading[1] = back_left;
            back_sensor = back_left;
            front_sensor = front_left;
        } else {
            ROS_INFO("Want to detect wall with back right sensor");
            previous_sensor_reading[1] = back_right;
            back_sensor = back_right;
            front_sensor = front_right;
        }
        bool back = false;
        ros::Rate loop_rate(10);

	ROS_INFO("back sensor: %d", back_sensor);

        while (!back) {
            ros::spinOnce();
            if(stop){
                stopRobot();
                while (stop) {
                    loop_rate.sleep();
                }
            }
            if (wallInFront()) return;
            if (prev_state == RIGHT_TURN) {
                back_sensor = back_left;
                front_sensor = front_left;
            } else {
                back_sensor = back_right;
                front_sensor = front_right;
            }
            if (abs(back_sensor - previous_sensor_reading[1]) > 20) back = true; //TODO Try different values for wall edge detection. Old value was 15
            ROS_INFO("back: %d", back);
            ROS_INFO("Value: %d", back_sensor);
            forward();
            loop_rate.sleep();
        }

        if (front_sensor > 25) {
            forward(15.0);
            if(prev_state == LEFT_TURN) setClientCall(RIGHT_TURN);
            else setClientCall(LEFT_TURN);
            forward(20);
        }
    }

    bool wallInFront() {
        int tresh_front = 15;//17;
        if ((forward_left < tresh_front &&
                forward_left >= 0) ||
                (forward_right < tresh_front && forward_right >= 0) ||
                (forward_left > 40 && forward_right < 20) ||
                (forward_right > 40 && forward_left < 20)) {
            return true;
        }
        return false;
    }

    double wallTooClose() {
        if ((front_left < 9 && front_left > 0 && back_left < 9 && back_left > 0) &&
                front_right < 9 && front_right > 0 && back_right < 9 && back_right > 0)
            return 0.0;
        else if (front_left < 9 && front_left > 0 && back_left < 9 && back_left > 0) {
            return -0.314;
        } else if (front_right < 9 && front_right > 0 && back_right < 9 && back_right > 0) {
            return 0.314;
        }
    }

    //Checks the ir sensors which decides what state to use
    int chooseState() {
        int s;

        //Checks if robot is close to a wall do turn, else follow wall or go forward
        if (wallInFront()) {
            if (front_right > front_left ||
                       back_right > back_left) {
                    s = RIGHT_TURN;
            } else
                    s = LEFT_TURN;
        } else { //If no wall seen in front of robot
             if (front_left < front_right &&
                   back_left < back_right &&
                   front_left < 25 &&
                   back_left < 25) {
                s = FOLLOW_LEFT;
        } else if (front_right < front_left &&
                      back_right < back_left &&
                      front_right < 25 &&
                      back_right < 25) {
                s = FOLLOW_RIGHT;
        } else
                s = FORWARD;
        }
	//ROS_INFO("State: %d", s);
        //Decides the state depending on the state transition
        if (previous_state == FOLLOW_LEFT && s == FORWARD) {
             s = TWO_LEFT;
        } else if (previous_state == LEFT_TURN && s != FOLLOW_RIGHT) {//(s == FORWARD || s == FOLLOW_RIGHT)) {
            ros::spinOnce();
            ROS_INFO("SMALL_EDGE_TURN left");
            if (front_right > 25 || back_right > 25) {
                s = SMALL_EDGE_TURN;
            }
        } else if (previous_state == RIGHT_TURN && s != FOLLOW_LEFT) {//(s == FORWARD || s == FOLLOW_LEFT)) {
            ros::spinOnce();
            ROS_INFO("SMALL_EDGE_TURN right");
            if (front_left > 25 || back_left > 25) {
                s = SMALL_EDGE_TURN;
            }
        }

        /*if (s != previous_state) {
            changeState(s);
        }*/

	ROS_INFO("State: %d", s);

        return s;
    }

    bool robotReady() {
        if (front_left != 0 ||
                front_right != 0 ||
                back_left != 0 ||
                back_right != 0 ||
                forward_left != 0 ||
                forward_right != 0) return true;
        else false;
    }

private:
    ros::ServiceClient turn_client;
    ros::ServiceClient follow_client;
    robot_msgs::MakeTurn srv_turn;
    robot_msgs::FollowWall srv_follow;
    ros::ServiceServer stop_server;
    geometry_msgs::Twist msg;
    int delta_encoder_left;
    int delta_encoder_right;
};

 int main(int argc, char **argv)
 {
    ros::init(argc, argv, "maze_follower");

    MazeController mc = MazeController();

    ros::Rate loop_rate(10);

    //Initial state
    state = FORWARD;

    //Waits for the robot to get non-zero values from Ir sensors
    ros::spinOnce();
    while (!mc.robotReady()) {
        ros::spinOnce();
    }

   while (ros::ok())
   {
        ros::spinOnce();
        if(mc.stop){
            mc.stopRobot();
        }
        else{
            //Returns the state the robot is in depending on the ir sensor readings
            state = mc.chooseState();

            //ROS_INFO("State: %d", state);

            //Do action depending on the state
            switch (state) {
                case FORWARD:
                    mc.forward();
                    break;

                case LEFT_TURN:
                    mc.setClientCall(state);
                    break;

                case RIGHT_TURN:
                    mc.setClientCall(state);
                    break;

                case FOLLOW_LEFT:
                    if (state != mc.previous_state) {
                        mc.previous_sensor_reading[0] = front_left;
                        mc.previous_sensor_reading[1] = back_left;
                    }
                    if (mc.checkSensorsDistanceLeft())
                        mc.setClientCall(state);
                    else {
                        mc.forward(22.0);
                    }
                    break;

                case FOLLOW_RIGHT:
                    if (state != mc.previous_state) {
                        mc.previous_sensor_reading[0] = front_right;
                        mc.previous_sensor_reading[1] = back_right;
                    }
                    if (mc.checkSensorsDistanceRight()) mc.setClientCall(state);
                    else {
                        mc.forward(22.0);
                    }
                    break;

                case TWO_LEFT:
                    ros::spinOnce();
                    if(front_left > 25 || back_left > 25)  {
                            mc.forward(22.0);
                            mc.setClientCall(LEFT_TURN);
                            mc.checkSensorsTurn();
                            ros::spinOnce();
                            if(mc.stop){
                                mc.stopRobot();
                                while (mc.stop) {
                                    loop_rate.sleep();
                                }
                            }
                            if (front_left > 25) {
                                mc.forward(12.0);
                                mc.setClientCall(LEFT_TURN);
                            }
                    }
            break;

                case SMALL_EDGE_TURN:
                    mc.checkBackSensorTurn(mc.previous_state);
            }

            mc.previous_state = state;


        }
        loop_rate.sleep();
   }

   return 0;
 }
