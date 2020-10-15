#ifndef maccepa
#define maccepa

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>

#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include "std_msgs/Float64MultiArray.h"

/* number of state variables */
const int N = 2;
/* number of output variables */
const int M = 2;
/* number of input variables */
const int U = 2;

enum Mode {
    MODE_POSITION = 1,
    MODE_SPEED = 2
};

namespace gazebo {

    class maccepaPlugin : public ModelPlugin {

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void init();
        void cardioid_motor_callback(const std_msgs::Float64::ConstPtr &msg);
        void pretension_motor_callback(const std_msgs::Float64::ConstPtr &msg);
        void motor_ref_limit(double &q1_ref_next, double &q2_ref_next);
        void motor_torque_limit(double &u);
        void system_update(double *x, double *y, double *u);
        void updateTorques();
        void updateMotorState(double &e, double &q, double &q_dot, double *x, double *y, double *u,
                              double q_ref_prev, double q_prev, double e_prev, double torque_external);
        double calculate_spring_force(double motor1_angle, double motor2_angle, double output_shaft_angle, double &force_arm);
        void OnReset();
        void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
        bool verbose = false;
        Mode mode = MODE_POSITION;

        /* State vectors
           for each motor :
           -) 1st state: motor output shaft angle
           -) 2nd state: motor output shaft angular speed */
        /* state vector motor1 */
        double x1_k[N];
        /* state vector motor2 */
        double x2_k[N];


        /* Output vectors
           Motor output angle value is the only output considered (1st output) */

        /* output vector motor 1 */
        double y1_k[M];
        /* output vector motor 2 */
        double y2_k[M];

        /* input vector motor 1 */
        double u1_k[U];
        /* input vector motor 2 */
        double u2_k[U];

        /* link physical parameters (MACCEPA 2.0)*/
        double k; /* spring constant [H/m] */
        double B; /* B segment length [m] */
        double C; /* C segment length [m] */
        double R; /* cam disk radius [m] */
        double R_pretension; /* motor 2 spool radius [m] */
        double initial_pretension; /* pretension motor initial pretension [rad] */


        /* Pointer to the model */
        physics::ModelPtr model;

        /* Pointer to output shaft joint */
        physics::JointPtr joint;
        /* Output shaft joint name */
        std::string joint_name;

        /* Pointer to cardioid motor joint */
        physics::JointPtr cardioid_motor_joint;

        /* Pointer to pretension motor joint */
        physics::JointPtr pretension_motor_joint;
        
        /* PID parameters in discrete domain */
        double Kp;
        double Ki;
        double Kd;

        /* motor 1 variables */
        /* motor 1 current position */
        double q1;
        /* motor 1 current velocity */
        double q1_dot;
        /* motor 1 previous position */
        double q1_prev;
        /* motor 1 previous velocity */
        double q1_dot_prev;
        /* motor 1 current reference position */
        double q1_ref;
        /* motor 1 current reference velocity */
        double q1_dot_ref;
        /* motor 1 previous reference position */
        double q1_ref_prev;
        /* motor 1 previous reference velocity */
        double q1_dot_ref_prev;
        /* motor 1 (k-1)-th step error (error on previous time moment) */
        double e1_prev;
        /* motor 1 external torque (second input) */
        double torque1_external;

        /* motor 2 variables */
        /* motor 2 current position */
        double q2;
        /* motor 2 current velocity */
        double q2_dot;
        /* motor 2 previous position */
        double q2_prev;
        /* motor 2 previous velocity */
        double q2_dot_prev;
        /* motor 2 current reference position */
        double q2_ref;
        /* motor 2 current reference velocity */
        double q2_dot_ref;
        /* motor 2 previous reference position */
        double q2_ref_prev;
        /* motor 2 previous reference velocity */
        double q2_dot_ref_prev;
        /* motor 2 (k-1)-th step error (error on previous time moment) */
        double e2_prev;
        /* motor 2 external torque (second input) */
        double torque2_external;

        /* output shaft angle */
        double output_shaft_angle;
        double output_shaft_angle_prev;

        /* output shaft torque */
        double output_shaft_torque;

        /* Variables for Gazebo topics  */
        /* node variable */
        transport::NodePtr node;

        ros::Subscriber cardioid_sub;
        ros::Subscriber pretension_sub;
        ros::NodeHandle n;

        ros::Publisher test_pub;
        /* Publisher for motors state*/
        ros::Publisher motors_pub;
        bool is_motors_state_published = false;

        std_msgs::Float64MultiArray msg_test;
        sensor_msgs::JointState motors_msg;

        /* Pointer to the update event connection */
        event::ConnectionPtr updateConnection;
        event::ConnectionPtr resetConnection;

        /* Motors limits (negative numbers - no limit) */
        double motors_angle_limit = -1.0;
        double motors_velocity_limit = -1.0;
        double motors_torque_limit = -1.0;
    };

    /* Register this plugin with the simulator */
    GZ_REGISTER_MODEL_PLUGIN(maccepaPlugin)

}


#endif


