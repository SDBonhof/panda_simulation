/*
 * File:   AIC_controller.cpp
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on April 14th, 2019
 *
 * This node allows to control the 7DOF Franka Emika Panda robot arm through
 * the new promisin theory called Active Inference proposed by Karl Friston.
 *
 * The robot moves to the desired position specified in desiredPos performing
 * free-energy minimization and actiove inference using gradient descent.
 * The robot is equipped with proprioceptive sensors for joints position and
 * velocity and a camera for the end-effector pose estimation.
 * The control is in joint space.
 *
 */

#include "AIC.h"

// Constant for class AIC constructor to define which robot to control
const int robot = 1;
double currentArmGoal;
Eigen::Matrix<double, 7, 1>  mu_dRight, mu_dGrab, mu_dCenter, mu_dRelease, mu_dDrop, mu_dCratePreDrop, mu_dCrateDrop, mu_dHandshakeDown, mu_dHandshakeUp, mu_dHandshakeDownDown;


void setGoalCallback(const std_msgs::Float64::ConstPtr& msg){
    //mu_d << msg;
  for( int i = 0; i < 7; i++ ) {
    currentArmGoal = msg->data;
     // ROS_INFO("Current arm goal: %f", currentArmGoal);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AIC_controller_single_node");
  ros::NodeHandle nh;
  ros::Subscriber goalSub = nh.subscribe("/franka_arm/AIC_controller/armGoal", 5, setGoalCallback);

  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;
  int cycles = 0;
  // Variable for desired position, set here the goal for the Panda for each joint
  std::vector<double> desiredPos1(7), desiredPos2(7), desiredPos3(7), mu_dGrab(7), mu_dCenter(7), mu_dDrop(7), mu_dCratePreDrop(7), mu_dCrateDrop(7), mu_dHandshakeDown(7), mu_dHandshakeUp(7), mu_dHandshakeDownDown(7);

  desiredPos1[0] = 1;
  desiredPos1[1] = 0.5;
  desiredPos1[2] = 0.0;
  desiredPos1[3] = -2;
  desiredPos1[4] = 0.0;
  desiredPos1[5] = 2.5;
  desiredPos1[6] = 0.0;

  desiredPos2[0] = 0.0;
  desiredPos2[1] = 0.2;
  desiredPos2[2] = 0.0;
  desiredPos2[3] = -1.0;
  desiredPos2[4] = 0.0;
  desiredPos2[5] = 1.2;
  desiredPos2[6] = 0.0;

  desiredPos3[0] = -1;
  desiredPos3[1] = 0.5;
  desiredPos3[2] = 0.0;
  desiredPos3[3] = -1.2;
  desiredPos3[4] = 0.0;
  desiredPos3[5] = 1.6;
  desiredPos3[6] = 0;

// {0.1628, 0.4644, -0.3587, -1.5428, 2.5763, 2.6365, 1.4702}
  mu_dGrab[0] = 0.1628;
  mu_dGrab[1] = 0.4644;
  mu_dGrab[2] = -0.3587;
  mu_dGrab[3] = -1.5428;
  mu_dGrab[4] = 2.5763;
  mu_dGrab[5] = 2.6365;
  mu_dGrab[6] = 1.4702;

  mu_dCenter[0] = 0.0;
  mu_dCenter[1] = -0.2170;
  mu_dCenter[2] = -0.0995;
  mu_dCenter[3] = -2.2655;
  mu_dCenter[4] = -0.0792;
  mu_dCenter[5] = 1.3512;
  mu_dCenter[6] = 0.7833;

  
  mu_dDrop[0] = 0.3426;
  mu_dDrop[1] = 0.5294;
  mu_dDrop[2] = 0.8897;
  mu_dDrop[3] = -1.748;
  mu_dDrop[4] = -0.01367;
  mu_dDrop[5] = 3.3494;
  mu_dDrop[6] = 0.4832;

  mu_dCratePreDrop[0] = -0.965;
  mu_dCratePreDrop[1] = 0.721;
  mu_dCratePreDrop[2] = -1.225;
  mu_dCratePreDrop[3] = -0.546;
  mu_dCratePreDrop[4] = 0.841;
  mu_dCratePreDrop[5] = 1.441;
  mu_dCratePreDrop[6] = 0.801;

  mu_dCrateDrop[0] = -1.256;
  mu_dCrateDrop[1] = 1.236;
  mu_dCrateDrop[2] = -0.485;
  mu_dCrateDrop[3] = -0.479;
  mu_dCrateDrop[4] = 0.441;
  mu_dCrateDrop[5] = 1.932;
  mu_dCrateDrop[6] = 0.490;

  mu_dHandshakeDown[0] = 0.038889695553926;
  mu_dHandshakeDown[1] = 0.6381180182739541;
  mu_dHandshakeDown[2] = -0.044217956547151556;
  mu_dHandshakeDown[3] = -1.3935411488373644;
  mu_dHandshakeDown[4] = 0.16626673391130237;
  mu_dHandshakeDown[5] = 2.8946352998415636;
  mu_dHandshakeDown[6] = 2.266841678474512;

  mu_dHandshakeUp[0] = 0.05362400182343001;
  mu_dHandshakeUp[1] = 0.6357887277015729;
  mu_dHandshakeUp[2] = -0.04648326963247585;
  mu_dHandshakeUp[3] = -0.7568542687511977;
  mu_dHandshakeUp[4] = 0.16622033984131282;
  mu_dHandshakeUp[5] = 2.8943688559532164;
  mu_dHandshakeUp[6] = 2.257162653460272;

  mu_dHandshakeDownDown[0] = 0.05827857613458967;
  mu_dHandshakeDownDown[1] = 1.0067789018259017;
  mu_dHandshakeDownDown[2] = -0.046631567179111;
  mu_dHandshakeDownDown[3] = -1.4198834604492843;
  mu_dHandshakeDownDown[4] = 0.16625964215066605;
  mu_dHandshakeDownDown[5] = 3.3757762997652367;
  mu_dHandshakeDownDown[6] = 2.263580544644759;


  // Object of the class AIC which will take care of everything
  AIC AIC_controller(robot);
  // Set desired position in the AIC class
  AIC_controller.setGoal(desiredPos1);

  // Main loop
  ros::Rate rate(1000);
  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(AIC_controller.dataReady()==1)){
      AIC_controller.minimiseF();
      cycles ++;
      // Choose goal
      if (currentArmGoal == 0) {
        AIC_controller.setGoal(mu_dCenter);
        // mu_d = mu_dCenter;
      }
      if (currentArmGoal == 1) {
        AIC_controller.setGoal(mu_dGrab);
        // mu_d = mu_dGrab;
      }
      if (currentArmGoal == 2) {
        AIC_controller.setGoal(mu_dDrop);
        // mu_d = mu_dDrop;
      }
      if (currentArmGoal == 3) {
        AIC_controller.setGoal(mu_dCratePreDrop);
        // mu_d = mu_dCratePreDrop;
      }
      if (currentArmGoal == 4) {
        AIC_controller.setGoal(mu_dCrateDrop);
        // mu_d = mu_dCrateDrop;
      }
      if (currentArmGoal == 5) {
        AIC_controller.setGoal(mu_dHandshakeDown);
        // mu_d = mu_dHandshakeDown;
      }
      if (currentArmGoal == 6) {
        AIC_controller.setGoal(mu_dHandshakeUp);
        // mu_d = mu_dHandshakeUp;
      }
      if (currentArmGoal == 7) {
        AIC_controller.setGoal(mu_dHandshakeDownDown);
        // mu_d = mu_dHandshakeDownDown;
      }
      // if (cycles == 6000){
      //   //AIC_controller.cameraFaultON();
      //   AIC_controller.setGoal(desiredPos2);
      // }

      // if (cycles == 12000){
      //   //AIC_controller.cameraFaultOFF();
      //   AIC_controller.setGoal(desiredPos3);
      // }

      // if (cycles == 18000){
      //   //AIC_controller.cameraFaultOFF();
      //   AIC_controller.setGoal(desiredPos2);
      // }

      // if (cycles == 24000){
      //   //AIC_controller.cameraFaultOFF();
      //   AIC_controller.setGoal(desiredPos1);
      //   cycles = 0;
      // }
    }
    else
      count ++;

    rate.sleep();
  }
  return 0;
}
