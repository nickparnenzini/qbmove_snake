#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <iostream>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include "position_stiffness_request.pb.h"
#include "pos_current_echo_request.pb.h"

#define N 12 //snake dimension

typedef const boost::shared_ptr<const position_stiffness_creator_msgs::msgs::PositionStiffnessRequest> PositionStiffnessRequestPtr;

typedef const boost::shared_ptr<const pos_current_echo_creator_msgs::msgs::PosCurrentEchoRequest> PosCurrentEchoRequestPtr; 
  

/////////////////////////////////////////////////
// Function is called everytime a message is received.
 
void cb(PosCurrentEchoRequestPtr &msg) 
{
   
   // Dump the message contents to stdout.
   std::cout << "OK!" << "\n"; 
 /*  std::cout << "Output shaft position echo: " << msg->pos_out_shaft() << "\n";
   std::cout << "Motor 1 position echo: " << msg->pos_mot_1() << "\n";
   std::cout << "Motor 2 position echo: " << msg->pos_mot_2() << "\n";
   std::cout << "Motor 1 current echo: " << msg->curr_mot_1() << "\n";
   std::cout << "Motor 2 current echo: " << msg->curr_mot_2() << "\n"; */

}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
      // Load gazebo
      gazebo::setupClient(argc, argv);

      // Create our nodes for communication
      gazebo::transport::NodePtr node(new gazebo::transport::Node());
      node->Init(); 

      /* input parameters */
      double lambda_v = atof(argv[1]); 

      double lambda_o = atof(argv[2]);

      /* vertical displacement */
      double displacement_v = 3.14/lambda_v;  

      /* horizontal displacement */
      double displacement_o = 3.14/lambda_o;

      /* omega (vertical and horizontal value are the same) */
      double omega = atof(argv[3]); //1.5*3.14;

      /* amplitude of vertical sinusoids (in degrees) */
      double amplitude_v = atof(argv[4]);  //60; 

      /* amplitude of horizontal sinusoids (in degrees) */
      double amplitude_o = atof(argv[5]);

      /* offset of vertical sinusoids (in degrees) */
      double offset_v = atof(argv[6]);

      /* offset of horizontal sinusoids (in degrees) */
      double offset_o = atof(argv[7]);

      /* stiffness for vertical cubebots (inpair cubebots), in degrees. The first cubebot is numbered as 0)*/
      double stiffness_v = atof(argv[8]);

      /* stiffness for horizontal cubebots (pair cubebots), in degrees */
      double stiffness_o = atof(argv[9]);

      /* second displacement value given to horizontal cubebots */
      double delta = atof(argv[10]); 

      /* period used for sinusoids */
      double Ts = 0.01;

      gazebo::transport::SubscriberPtr subscriber_vector[N]; 

      gazebo::transport::PublisherPtr publisher_vector[N];

      /* string used to initialize publisher topics */
      std::string publisher_string = ""; 

      /* string used to initialize subscriber topics */
      std::string subscriber_string = "";

      /* variable used for sinusoids values */
      double dt = 0.0 ;

      /* displacement variable for each sinusoid */
      double displacement = 0.0;

      double sin_value = 0.0; 

      double stiffness = 0.0;

      /* vectors initialization */
      for(int i=0; i<N; i++)
      {
              publisher_string = "qbmove_" + boost::lexical_cast<std::string>(i+1) + "/position_stiffness/command"; 

              std::cout << publisher_string << "\n";

              publisher_vector[i] = node->Advertise<position_stiffness_creator_msgs::msgs::PositionStiffnessRequest>(publisher_string);

              publisher_vector[i]->WaitForConnection();

              subscriber_string = "qbmove_" + boost::lexical_cast<std::string>(i+1) + "/pos_current/echo";

              subscriber_vector[i] = node->Subscribe(subscriber_string, cb);

      }

      position_stiffness_creator_msgs::msgs::PositionStiffnessRequest msg_pub; 

            
      while (true)
      {  
               dt = dt + Ts; 

               for(int i=0; i<N; i++)
               { 

                   if(i%2 != 0)  //vertical qbmove
                   {
                      stiffness = stiffness_v; 

                      displacement = i*displacement_v; 

                      sin_value = offset_v + amplitude_v*sin( omega*dt + displacement );                                          

                   }
                   else //horizontal qbmove
                   {    
                      stiffness = stiffness_o;  

                      displacement = i*displacement_o; 

                      sin_value = offset_o + amplitude_o*sin( omega*dt + displacement + delta );                              

                   }

                   msg_pub.set_stiffness(stiffness);

                   msg_pub.set_position(sin_value);
                   
                   publisher_vector[i]->Publish(msg_pub);

               }
         
               gazebo::common::Time::MSleep(10);

      }
    
      return 0; 
 
}
