#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

using namespace ros;
using namespace std;

#define PI 3.14159

class Test
{
private:
  NodeHandle nh;
  double ori_x, ori_y, ori_fi;
  double dst_x, dst_y, dst_fi;
  double theta, alpha, beta, lo;
  double v, w;
  double k_lo, k_alpha, k_beta;
  double lo_dot, alpha_dot, beta_dot;
  double x_dot, y_dot, fi_dot;
  geometry_msgs::Twist msg;
  std_msgs::Float32MultiArray msg_float;
  double delta_t;

public:
  Publisher pub = nh.advertise<geometry_msgs::Twist> ("/turtle1/cmd_vel", 1);
  Publisher pub2 = nh.advertise<std_msgs::Float32MultiArray> ("/turtle_pos", 1);
  //Rate rrate(10);

  Test(){
    nh.getParam("/origin_x", ori_x);
    nh.getParam("/origin_y", ori_y);
    nh.getParam("/origin_fi", ori_fi);
    nh.getParam("/dst_x", dst_x);
    nh.getParam("/dst_y", dst_y);
    nh.getParam("/dst_fi", dst_fi);

    ori_fi = ori_fi * PI / 180;
    dst_fi = dst_fi * PI / 180;

    k_lo = 1;
    k_alpha = 5;
    k_beta = -0.1;

    delta_t = 0.001;

    v = 0;
    w = 0;

    x_dot = 0;
    y_dot = 0;
    fi_dot = 0;
    lo_dot = 0;
    alpha_dot = 0;
    beta_dot = 0;
  }
  
void Calculate_angles(){
    theta = atan2((ori_y-dst_y), (ori_x-dst_x));
    
    lo = sqrt(pow(dst_x-ori_x, 2) + pow(dst_y-ori_y, 2));
    alpha = theta + 3.14159 - ori_fi;
    beta = -(alpha) - ori_fi;

    v = k_lo * lo;
    w = k_alpha * alpha + k_beta * beta;

    //x_dot = cos(ori_fi) * v;
    //y_dot = sin(ori_fi) * v;
    //fi_dot = w;

    cout << "theta = " << theta*180/PI << endl;
/*
    cout << "lo = " << lo << endl;
*/    
    cout << "alpha = " << alpha*180/PI << endl;
    cout << "beta = " << beta*180/PI << endl;
    //cout << "fi = " << ori_fi << endl;

    cout << "v = " << v << endl;
    cout << "w = " << w << endl;
    
/*
    cout << "x_dot = " << x_dot << endl;
    cout << "y_dot = " << y_dot << endl;
    cout << "fi_dot = " << fi_dot << endl;
*/
    //cout << "lo_dot = " << lo_dot << endl;
    //cout << "alpha_dot = " << alpha_dot << endl;
    //cout << "beta_dot = " << beta_dot << endl;
  
  }

  void show_status(){
    cout << "position t = (" << ori_x << ", " << ori_y << ", " << ori_fi*180/PI << ")" << endl;
    //cout << "position t+1 = (" << ori_x + x_dot << ", " << ori_y + y_dot << ", " << ori_fi + fi_dot << ")" << endl;
    cout << "---------------------------------" <<  endl;
  } 

  void update_pos(){
    ori_x = ori_x + cos(ori_fi)*v*delta_t;
    ori_y = ori_y + sin(ori_fi)*v*delta_t;
    ori_fi = ori_fi + w*delta_t;
  }

  bool arrive(){
    if ((abs(ori_x - dst_x) < 0.1) && (abs(ori_y - dst_y) < 0.1)){
      return true;
    }
    else{
      return false;
    }
  }

  void pub_(){
    //cout << "called" << endl;
    msg.linear.x = -(sqrt(pow(ori_x, 2) + pow(ori_y, 2)) / 100);
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = ori_fi / 100;

    pub.publish(msg);
    //rrate.sleep();
  }

  void pub_pos(){
    msg_float.data.push_back(ori_x);
    msg_float.data.push_back(ori_y);
    msg_float.data.push_back(ori_fi);
    pub2.publish(msg_float);
    msg_float.data.clear();
  }
};


int main(int argc, char** argv){
  init(argc, argv, "turtle");
  //NodeHandle nh;
  //Rate rate(10);
  Test tt;
  Rate rate(5);

  //tt.Calculate_angles();
  //tt.show_status();
/*
  for (int i = 0; i <= 9; i ++){
  if (tt.arrive() != true){
    tt.update_pos();
    tt.Calculate_angles();
    tt.show_status();
  }
  }
*/
  //while (1){
  while (tt.arrive() != true){
    tt.update_pos();
    tt.Calculate_angles();
    tt.show_status();
    tt.pub_();
    tt.pub_pos();
    rate.sleep();
    spinOnce();
  }
  //spin();
  
  return 0;
}
