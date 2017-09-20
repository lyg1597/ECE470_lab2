#include "lab2pkg/lab2.h" 
#define PI 3.14159265359
#define SPIN_RATE 20  /* Hz */

//arrays defining Waypoints
double home[]={120*PI/180,-90*PI/180,90*PI/180,-90*PI/180,-90*PI/180,0};

// double arr11[]={120*PI/180,-56*PI/180,124*PI/180,-158*PI/180,-90*PI/180,0};
// double arr12[]={120*PI/180,-64*PI/180,123*PI/180,-148*PI/180,-90*PI/180,0};
// double arr13[]={120*PI/180,-72*PI/180,120*PI/180,-137*PI/180,-90*PI/180,0};
// double arr14[]={120*PI/180,-78*PI/180,115*PI/180,-125*PI/180,-90*PI/180,0};

double arr11[]={129.27*PI/180,-56.85*PI/180,131.41*PI/180,-172.05*PI/180,-87.83*PI/180,0};
double arr12[]={129.1*PI/180,-65.6*PI/180,121.54*PI/180,-143.82*PI/180,-86.41*PI/180,0};
double arr13[]={130.34*PI/180,-72.03*PI/180,117.63*PI/180,-132.04*PI/180,-89.44*PI/180,0};
double arr21[]={142.16*PI/180,-44.4*PI/180,94.11*PI/180,-140.23*PI/180,-87.75*PI/180,0};
double arr22[]={141.86*PI/180,-50.36*PI/180,94.48*PI/180,-136.08*PI/180,-87.01*PI/180,0};
double arr23[]={141.93*PI/180,-54.91*PI/180,92.49*PI/180,-128.93*PI/180,-86.71*PI/180,0};
double arr31[]={162.49*PI/180,-63.2*PI/180,143.76*PI/180,-173.32*PI/180,-90.1*PI/180,0};
double arr32[]={161.17*PI/180,-74.79*PI/180,144.95*PI/180,-164.91*PI/180,-89.19*PI/180,0};
double arr33[]={161*PI/180,-86.76*PI/180,140.72*PI/180,-145.74*PI/180,-89.03*PI/180,0};
double arrH1[]={132.12*PI/180,-82.81*PI/180,104.56*PI/180,-109.19*PI/180,-93.92*PI/180,0};
double arrH2[]={144.55*PI/180,-62.59*PI/180,76.05*PI/180,-100.05*PI/180,-93.2*PI/180,12.39*PI/180.0};
double arrH3[]={165.1*PI/180,-105.22*PI/180,120.28*PI/180,-100.57*PI/180,-91.9*PI/180,32.91*PI/180.0};

// array to define final velocity of point to point moves.  For now slow down to zero once 
// each point is reached
double arrv[]={0,0,0,0,0,0};

//vectors to be used to publish commands to UR3 ROS Driver (ece470_ur3_driver)
std::vector<double> QH (home,home+sizeof(home) / sizeof(home[0]));

// std::vector<double> Q11 (arr11,arr11+sizeof(arr11) / sizeof(arr11[0]));
// std::vector<double> Q12 (arr12,arr12+sizeof(arr12) / sizeof(arr12[0]));
// std::vector<double> Q13 (arr13,arr13+sizeof(arr13) / sizeof(arr13[0]));
// std::vector<double> Q14 (arr14,arr14+sizeof(arr14) / sizeof(arr14[0]));

std::vector<double> Q11 (arr11,arr11+sizeof(arr11) / sizeof(arr11[0]));
std::vector<double> Q12 (arr12,arr12+sizeof(arr12) / sizeof(arr12[0]));
std::vector<double> Q13 (arr13,arr13+sizeof(arr13) / sizeof(arr13[0]));
std::vector<double> Q21 (arr21,arr21+sizeof(arr21) / sizeof(arr21[0]));
std::vector<double> Q22 (arr22,arr22+sizeof(arr22) / sizeof(arr22[0]));
std::vector<double> Q23 (arr23,arr23+sizeof(arr23) / sizeof(arr23[0]));
std::vector<double> Q31 (arr31,arr31+sizeof(arr31) / sizeof(arr31[0]));
std::vector<double> Q32 (arr32,arr32+sizeof(arr32) / sizeof(arr32[0]));
std::vector<double> Q33 (arr33,arr33+sizeof(arr33) / sizeof(arr33[0]));
std::vector<double> QH1 (arrH1,arrH1+sizeof(arrH1) / sizeof(arrH1[0]));
std::vector<double> QH2 (arrH2,arrH2+sizeof(arrH2) / sizeof(arrH2[0]));
std::vector<double> QH3 (arrH3,arrH3+sizeof(arrH3) / sizeof(arrH3[0]));

std::vector<double> v (arrv,arrv+sizeof(arrv) / sizeof(arrv[0]));

// creating an array of these vectors allows us to iterate through them
// and programatically choose where to go.
std::vector<double> Q [3][4] = {
    {Q11, Q12, Q13, QH1},
    {Q21, Q22, Q23, QH2},
    {Q31, Q32, Q33, QH3}
};

// std::vector<double> high[3]={QH1,QH2,QH3};

// Global bool variables that are assigned in the callback associated when subscribed 
// to the "ur3/position" topic
bool isReady=1;
bool pending=0;

// Whenever ur3/position publishes info this callback function is run.
void position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady; // When isReady is True the robot arm has made it to its desired position
						  // and is ready to be told to go to another point if desired.
	pending=msg->pending; // pending is the opposite of isReady, pending is true until a new position is reached
//	ROS_INFO("Debug isRdy = %d, pending = %d",(int)isReady,(int)pending);
}


int move_arm(	ros::Publisher pub_command , ros::Rate loop_rate, std::vector<double> dest, float duration)
{
    int error = 0;
    return error;
}

int move_block(ros::Publisher pub_command ,
                ros::Rate loop_rate,
                ros::ServiceClient srv_SetIO,
                ur_msgs::SetIO srv,
                int start_loc,
                int start_height,
                int end_loc,
                int end_height)
{
    int error = 0;
    return error;
}

int main(int argc, char **argv)
{
    
	int inputdone = 0;
	int Loopcnt = 0;
	int startPos = 0;
	int endPos = 0;
	int midPos = 0;
//initialization & variable definition
	ros::init(argc, argv, "lab2node");	//initialzation of ros required for each Node.
	ros::NodeHandle nh;				//handler for this node.
	
	//initialized publisher ur3/command, buffer size of 10.
	ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	// initialize subscriber to ur3/position and call function position_callback each time data is published
	ros::Subscriber sub_position=nh.subscribe("ur3/position",1,position_callback);
	
	ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
	ur_msgs::SetIO srv;

	ece470_ur3_driver::command driver_msg;

	std::string inputString;
	while (!inputdone) {
		std::cout << "Enter start point <Either 1 2 or 3>";
		std::getline(std::cin, inputString);
		std::cout << "You entered " << inputString << "\n";
		if (inputString == "1") {
			inputdone = 1;
			startPos=0;
		} else if (inputString == "2") {
			inputdone = 1;
			startPos=1;
		} else if (inputString == "3") {
			inputdone = 1;
			startPos=2;
		} else {
			std::cout << "Please just enter the character 1 2 or 3\n\n";
		}
	}

	inputdone=0;
	while (!inputdone) {
		std::cout << "Enter start point <Either 1 2 or 3>";
		std::getline(std::cin, inputString);
		std::cout << "You entered " << inputString << "\n";
		if (inputString == "1") {
			inputdone = 1;
			endPos=0;
		} else if (inputString == "2") {
			inputdone = 1;
			endPos=1;
		} else if (inputString == "3") {
			inputdone = 1;
			endPos=2;
		} else {
			std::cout << "Please just enter the character 1 2 or 3\n\n";
		}
	}

	midPos=6-startPos-endPos;

	while(!ros::ok()){};	//check if ros is ready for operation
		
	ROS_INFO("sending Goals");

	ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
	Loopcnt = 1; 
	while(Loopcnt > 0) {
		driver_msg.destination=QH;  // Set desired position to move home 
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		ROS_INFO("sending Goals 0");
		driver_msg.destination=Q[startPos][3];
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		ROS_INFO("sending Goals 1");
		driver_msg.destination=Q[startPos][2];
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		srv.request.fun = 1;
		srv.request.pin = 0;  //Digital Output 0
		srv.request.state = 1.0; //Set DO0 on
		if (srv_SetIO.call(srv)) {
			ROS_INFO("True: Switched Suction ON");
		} else {
			ROS_INFO("False");
		}

		ROS_INFO("sending Goals 2");
		driver_msg.destination=Q[startPos][1];
		driver_msg.duration=2.0;
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		srv.request.fun = 1;
		srv.request.pin = 0; // Digital Output 0
		srv.request.state = 0.0; //Set DO0 off
		if (srv_SetIO.call(srv)) {
			ROS_INFO("True: Switched Suction OFF");
		} else {
			ROS_INFO("False");
		}

		ROS_INFO("sending Goals 3");
		driver_msg.destination=Q[startPos][0];
		driver_msg.duration=1.0;
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
		Loopcnt--;
	}

	return 0;
}
