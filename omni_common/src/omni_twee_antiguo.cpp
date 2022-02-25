#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include "PhantomButtonEvent.h"
#include "OmniFeedback.h"
#include <pthread.h>

// Twee
# include <stdlib.h>
# include <ctype.h>
# include <string.h>


float prev_time;


struct OmniState
{
    hduVector3Dd position;  //3x1 vector of position
    hduVector3Dd velocity;  //3x1 vector of velocity
    hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
    hduVector3Dd inp_vel2;  
    hduVector3Dd inp_vel3;  
    hduVector3Dd out_vel1;  
    hduVector3Dd out_vel2;  
    hduVector3Dd out_vel3;
    hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity 
    hduVector3Dd pos_hist2; 
    hduVector3Dd rot;
    hduVector3Dd joints;
    hduVector3Dd force;     //3 element double vector force[0], force[1], force[2]
    float thetas[7];
    //int buttons[2];
    int twee_button;
    //int buttons_prev[2];
    bool lock;
    hduVector3Dd lock_pos;
};


class PhantomROS {

	public:
	ros::NodeHandle n;
	ros::Publisher omni_pose_publisher;

tf::StampedTransform origin;
    ros::Publisher button_publisher;
	ros::Subscriber haptic_sub;
    std::string omni_name;
    std::string sensable_frame_name;
    std::string link_names[7];

    OmniState *state;
    tf::TransformBroadcaster br;

PhantomROS():n("~") {};

    void init(OmniState *s) 
    {
		std::string str;
		double_t x, y, z;
		tf::Transform tf_origin;
		n.getParam("origin/position", str);
		std::stringstream ss(str);
		ss >> x >> y >> z;
		ss.str("");
		ss.clear();
		tf_origin.setOrigin(tf::Vector3(x, y, z));
		n.getParam("origin/orientation", str);
		ss.str(str);
		ss >> x >> y >> z;
		tf::Vector3 w(x, y, z);
		tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
		if (w.length())
		  //~ q.setEuler(x, y, z);
		  q.setRotation(w.normalized(), w.length());
		tf_origin.setRotation(q);
		// ROTA 90º ALREDEDOR DE X PARA HACER QUE Z SEA HACIA ARRIBA
		//~ tf::Transform rotX;
		  //~ rotX.setRotation(tf::Quaternion(tf::Vector3(1.0, 0.0, 0.0), 0.5*M_PI));
		//~ tf_origin = tf_origin*rotX;
		// ROTA 90º ALREDEDOR DE X PARA HACER QUE Z SEA HACIA ARRIBA
		origin.setData(tf_origin);
		n.getParam("origin/parent", str);
		origin.frame_id_ = str;
		n.getParam("origin/child", str);
		origin.child_frame_id_ = str;
		origin.stamp_ = ros::Time::now();origin.stamp_ = ros::Time::now();

        //Publish on NAME_pose
        std::ostringstream stream00;
        stream00 << omni_name << "/pose";
        std::string pose_topic_name = std::string(stream00.str());
		omni_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("pose", 100);
		
        //Publish button state on NAME_button
        std::ostringstream stream0;
        stream0 << omni_name << "/button";
        std::string button_topic = std::string(stream0.str());
        button_publisher = n.advertise<phantom_omni::PhantomButtonEvent>("button", 100);
	
		//Subscribe to NAME_force_feedback
        std::ostringstream stream01;
        stream01 << omni_name << "/force_feedback";
        std::string force_feedback_topic = std::string(stream01.str());
        haptic_sub = n.subscribe("force_feedback", 100, &PhantomROS::force_callback, this);

        //Frame of force feedback (NAME_sensable)
        std::ostringstream stream2;
        stream2 << omni_name << "/sensable";
        sensable_frame_name = std::string(stream2.str());

        for (int i = 0; i < 7; i++)
        {
            std::ostringstream stream1;
            stream1 << omni_name << "/link" << i;
            link_names[i] = std::string(stream1.str());
        }

        state = s;
		//state->buttons[0] = 0;
		//state->buttons[1] = 0;
		//state->buttons_prev[0] = 0;
		//state->buttons_prev[1] = 0;
        hduVector3Dd zeros(0, 0, 0);
		state->velocity = zeros;
        state->inp_vel1 = zeros;  //3x1 history of velocity
        state->inp_vel2 = zeros;  //3x1 history of velocity
        state->inp_vel3 = zeros;  //3x1 history of velocity
        state->out_vel1 = zeros;  //3x1 history of velocity
        state->out_vel2 = zeros;  //3x1 history of velocity
        state->out_vel3 = zeros;  //3x1 history of velocity
        state->pos_hist1 = zeros; //3x1 history of position 
        state->pos_hist2 = zeros; //3x1 history of position
		state->lock = true;
		state->lock_pos = zeros;
    }

    /*******************************************************************************
     ROS node callback.  
    *******************************************************************************/
  //    void force_callback(const geometry_msgs::WrenchConstPtr& wrench)
    void force_callback(const phantom_omni::OmniFeedbackConstPtr& omnifeed)
    {
      ////////////////////Some people might not like this extra damping, but it 
      ////////////////////helps to stabilize the overall force feedback. It isn't
      ////////////////////like we are getting direct impedance matching from the 
      ////////////////////omni anyway
        state->force[0] = omnifeed->force.x - 0.001*state->velocity[0];
		state->force[1] = omnifeed->force.y - 0.001*state->velocity[1];
		state->force[2] = omnifeed->force.z - 0.001*state->velocity[2];

		state->lock_pos[0] = omnifeed->position.x;
		state->lock_pos[1] = omnifeed->position.y;
		state->lock_pos[2] = omnifeed->position.z;
    } 

    void publish_omni_state()
    {

        geometry_msgs::PoseStamped omni_internal_pose;
        omni_internal_pose.header.frame_id = omni_name + "/ee_link";
        omni_internal_pose.header.stamp = ros::Time::now();
        omni_internal_pose.pose.position.x = 1e-3*state->position[0];
        omni_internal_pose.pose.position.y = 1e-3*state->position[1];
        omni_internal_pose.pose.position.z = 1e-3*state->position[2];
        omni_internal_pose.pose.orientation.x = state->rot[0];
        omni_internal_pose.pose.orientation.y = state->rot[1];
        omni_internal_pose.pose.orientation.z = state->rot[2];
        omni_pose_publisher.publish(omni_internal_pose);
/*
        if ((state->buttons[0] != state->buttons_prev[0]) or (state->buttons[1] != state->buttons_prev[1]))
        {
	    
	  if ((state->buttons[0] == state->buttons[1]) and (state->buttons[0] == 1))
	    {
	      state->lock = !(state->lock);
	    }
            phantom_omni::PhantomButtonEvent button_event;
            button_event.grey_button = state->buttons[0];
            button_event.white_button = state->buttons[1];
            state->buttons_prev[0] = state->buttons[0];
            state->buttons_prev[1] = state->buttons[1];
            button_publisher.publish(button_event);
        }
*/
        //Construct transforms
        origin.stamp_ = ros::Time::now();
        std::vector<tf::StampedTransform> tfList;
        //~ br.sendTransform(origin);
        tfList.push_back(origin);
        tf::Quaternion q_base, q_ee;
          q_base.setEuler(0.0, 0.0, 0.0);
          q_ee.setEuler(state->rot[0], state->rot[1], state->rot[2]);
        tf::Transform l_base, l_ee;
        l_base.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        l_base.setRotation(q_base);
        tfList.push_back(tf::StampedTransform(l_base, origin.stamp_, origin.child_frame_id_, omni_name + "/base_link"));

        l_ee.setOrigin(tf::Vector3(1e-3*state->position[0], 1e-3*state->position[1], 1e-3*state->position[2]));
        l_ee.setRotation(q_ee);
        tfList.push_back(tf::StampedTransform(l_ee, origin.stamp_, omni_name + "/base_link", omni_name + "/ee_link"));
        br.sendTransform(tfList);
    }
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
    // Twee ******
    // do not change these values, these are hardware dependent
	static const int maxTweeValue = 60;
	static const int minTweeValue = 35;
	// temporary variables that keep track of Twee state
	static int downCount = 0;
	static bool button = false;
	
	HDint buttonState;
    //*************
    OmniState *omni_state = static_cast<OmniState *>(pUserData);

    
	hdBeginFrame(hdGetCurrentDevice());
	//Twee ******
	hdGetIntegerv(HD_CURRENT_BUTTONS, &buttonState);
	
	//****
    //Get angles, set forces
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);      
	hdGetDoublev(HD_CURRENT_POSITION,      omni_state->position);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES,  omni_state->joints);

    hduVector3Dd vel_buff(0, 0, 0);
	vel_buff = (omni_state->position*3 - 4*omni_state->pos_hist1 + omni_state->pos_hist2)/0.002;  //mm/s, 2nd order backward dif
    omni_state->velocity = (.2196*(vel_buff+omni_state->inp_vel3)+.6588*(omni_state->inp_vel1+omni_state->inp_vel2))/1000.0-(-2.7488*omni_state->out_vel1+2.5282*omni_state->out_vel2 - 0.7776*omni_state->out_vel3);  //cutoff freq of 20 Hz
	omni_state->pos_hist2 = omni_state->pos_hist1;
	omni_state->pos_hist1 = omni_state->position;
	omni_state->inp_vel3 = omni_state->inp_vel2;
	omni_state->inp_vel2 = omni_state->inp_vel1;
	omni_state->inp_vel1 = vel_buff;
	omni_state->out_vel3 = omni_state->out_vel2;
	omni_state->out_vel2 = omni_state->out_vel1;
	omni_state->out_vel1 = omni_state->velocity;

	hdSetDoublev(HD_CURRENT_FORCE,         omni_state->force);
	
    //Get buttons
    /*int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;
	*/
	
	//Twee ***********
		int encoder = -1;
		//printf("\rButton = %d\t\t Angle = %02d", button, encoder);

	// start counting the number of ticks in which button 1 is pressed
	if (buttonState & HD_DEVICE_BUTTON_1)
	{
		downCount++;

		// if it is pressed for too long then consider that the clutch button is pressed (and keep counting)
		if (downCount > maxTweeValue * 3)
			button = true;
	}
	else
	{
		// if downCount > 0 button 1 has just been released, thus downCount contains the amount of ticks in
		// which button 1 has been pressed which is the encoder reading
		if (downCount > minTweeValue * 0.5f && downCount <= maxTweeValue * 3)
		{
			// shift minimum encoder value to 0s
			encoder = std::max(0, downCount - minTweeValue);
			encoder = std::min(encoder, maxTweeValue);
			
		}
		// zero out downCount to start over again 
		downCount = 0;
		button = false;
	}
printf("\rButton2 = %d\t\t Angle = %02d", button, encoder);

	// print state only when it makes sense
	if (encoder != -1 || button)
		//printf("\rButton2 = %d\t\t Angle = %02d", button, encoder);

//***********************
	hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
		hduPrintError(stderr, &error, "Error during main scheduler callback\n");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
    }

    float t[7] = {0., omni_state->joints[0], omni_state->joints[1], omni_state->joints[2]-omni_state->joints[1], 
                      omni_state->rot[0],    omni_state->rot[1],    omni_state->rot[2]};
    for (int i = 0; i < 7; i++)
        omni_state->thetas[i] = t[i];
    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
Automatic Calibration of Phantom Device - No character inputs
*******************************************************************************/
void HHD_Auto_Calibration()
{
   int calibrationStyle;
   int supportedCalibrationStyles;
   HDErrorInfo error;

   hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
   if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
   {
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		ROS_INFO("HD_CALIBRATION_ENCODER_RESE..\n\n");
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
   {
		calibrationStyle = HD_CALIBRATION_INKWELL;
		ROS_INFO("HD_CALIBRATION_INKWELL..\n\n");
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
   {
       calibrationStyle = HD_CALIBRATION_AUTO;
       ROS_INFO("HD_CALIBRATION_AUTO..\n\n");
   }

   do 
   {
	   hdUpdateCalibration(calibrationStyle);
	   ROS_INFO("Calibrating.. (put stylus in well)\n");
       if (HD_DEVICE_ERROR(error = hdGetError()))
       {
	       hduPrintError(stderr, &error, "Reset encoders reset failed.");
	       break;           
           }
   }   while (hdCheckCalibration() != HD_CALIBRATION_OK);

   ROS_INFO("\n\nCalibration complete.\n");
}

void *ros_publish(void *ptr)
{
   PhantomROS *omni_ros = (PhantomROS *) ptr;
   int publish_rate;
   omni_ros->n.param(std::string("publish_rate"), publish_rate, 1000);
   ros::Rate loop_rate(publish_rate);
   ros::AsyncSpinner spinner(2);
   spinner.start();

   while(ros::ok())
   {
       omni_ros->publish_omni_state();
       loop_rate.sleep();
   }
   return NULL;
}


int main(int argc, char** argv)
{
   ////////////////////////////////////////////////////////////////
   // Init Phantom
   ////////////////////////////////////////////////////////////////
   HDErrorInfo error;
   HHD hHD;
   hHD = hdInitDevice("Default PHANToM");
   
   if (HD_DEVICE_ERROR(error = hdGetError())) 
   {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        return -1;
   }

   ROS_INFO("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
   hdEnable(HD_FORCE_OUTPUT);
   hdStartScheduler(); 
   if (HD_DEVICE_ERROR(error = hdGetError()))
   {
       ROS_ERROR("Failed to start the scheduler");
       return -1;           
   }
   HHD_Auto_Calibration();

   ////////////////////////////////////////////////////////////////
   // Init ROS 
   ////////////////////////////////////////////////////////////////
	std::string devid = argv[1];
	devid.erase(0, devid.find('=') + 1);
   ros::init(argc, argv, devid);
   OmniState state;
   PhantomROS omni_ros;

omni_ros.omni_name = devid;
   omni_ros.init(&state);
   hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);
	
   ////////////////////////////////////////////////////////////////
   // Loop and publish 
   ////////////////////////////////////////////////////////////////
   pthread_t publish_thread;
   pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
   pthread_join(publish_thread, NULL);

   ROS_INFO("Ending Session....\n");
   hdStopScheduler();
   hdDisableDevice(hHD);

   return 0;
}


