#include <maccepa_plugin.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>

//################################################################
/* matrixes used for state space representation */

#define Ts 0.001

double Ad[N][N] = {{1.0, Ts},
                   {0,   1.0}
};
double Bd[N][U] = {{0.0, 0.0},
                   {0.0073, -0.0073}
};

double Cd[M][N] = {{1, 0}};

double Dd[M][U] = {{0, 0}};

//###############################################################

using namespace gazebo;
using namespace std;


/* sign function */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


void maccepaPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    int argc = 0;
    char **argv;
    ros::init(argc, argv, "maccepa_plugin");

    this->model = _parent;

    /* parse  plugin parameters */
    std::string joint_name = _sdf->GetElement("joint")->Get<string>();
    std::string joint_motor_name = _sdf->GetElement("joint_motor")->Get<string>();
    std::string command_topic = _sdf->GetElement("command_topic")->Get<string>();
    std::string stiffness_topic = _sdf->GetElement("stiffness_topic")->Get<string>();
    std::string echo_topic = _sdf->GetElement("pos_topic")->Get<string>();
    this->k = _sdf->GetElement("k")->Get<double>();
    this->B = _sdf->GetElement("B")->Get<double>();
    this->C = _sdf->GetElement("C")->Get<double>();
    this->R = _sdf->GetElement("R")->Get<double>();
    this->R_pretension = _sdf->GetElement("R_pretension")->Get<double>();


    /* bind joints */
    this->joint_name = joint_name;
    this->joint = this->model->GetJoint(joint_name);
    this->joint_motor1 = this->model->GetJoint(joint_motor_name);

    /* INITIALISATION */
    /* reference angle for motor 1 in radians */
    this->q1_ref = 0.0;
    this->q1_ref_prev = 0.0;
    this->q1_prev = 0.0;

    /* reference angle for motor 2 in radians */
    this->q2_ref = 0.0;
    this->q2_ref_prev = 0.0;
    this->q2_prev = 0.0;

    this->torque1_external = 0.0;
    this->torque2_external = 0.0;

    this->e1_prev = 0.0;
    this->e2_prev = 0.0;

    /* state initialization */
    for (int i = 0; i < N; i++) {
        x1_k[i] = 0.0;
        x2_k[i] = 0.0;
    }
    /* output initialization */
    for (int i = 0; i < M; i++) {
        y1_k[i] = 0.0;
        y2_k[i] = 0.0;
    }
    /* input initialization */
    for (int i = 0; i < U; i++) {
        u1_k[i] = 0.0;
        u2_k[i] = 0.0;
    }

    /* PID parameters in discrete domain at initialization */
    this->Kp = 686.0;
    this->Ki = 0*10.0;
    this->Kd = 12.0;


    /* create topic subscribers & publishers and bind them to callbacks*/
    command_sub = n.subscribe(command_topic, 1, &maccepaPlugin::position_Callback, this);
    stiffness_sub = n.subscribe(stiffness_topic, 1, &maccepaPlugin::pretension_Callback, this);
    test_pub = n.advertise<std_msgs::Float64MultiArray>("test", 500);

    /* bind onUpdate event */
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&maccepaPlugin::OnUpdate, this, _1));

}


/* updates motor 1 reference position */
void maccepaPlugin::position_Callback(const std_msgs::Float64::ConstPtr &msg) {
    this->q1_ref = msg->data;

    ROS_INFO("New motor value params: \n%f, %f", this->q1_ref, this->q2_ref);
}


/* updates motor 2 reference position */
void maccepaPlugin::pretension_Callback(const std_msgs::Float64::ConstPtr &msg) {
    this->q2_ref = msg->data;

    ROS_INFO("New motor value params: \n%f, %f", this->q1_ref, this->q2_ref);
}


/* limit motors reference position */
void maccepaPlugin::motor_ref_limit(double &q1_ref_next, double &q2_ref_next) {

    /* saturation limit for reference values */
    double q1_limit_value = 5*M_PI;
    double q2_limit_value = 0;

    /* check if q1_ref_next is over saturation limit*/
    if (q1_ref_next < -q1_limit_value) {
        q1_ref_next = -q1_limit_value;
    }
    if (q1_ref_next > q1_limit_value) {
        q1_ref_next = q1_limit_value;
    }

    /* check if q2_ref_next is over saturation limit*/
    if (q2_ref_next < q2_limit_value) {
        q2_ref_next = q2_limit_value;
    }

}


/* limit motors maximum torque */
void maccepaPlugin::motor_torque_limit(double &u) {

    /* saturation limit for torque values */
    double limit_value = 100;

    /* check if u value is over saturation limit*/
    if (u < -limit_value) {
        u = -limit_value;
    }

    if (u > limit_value) {
        u = limit_value;
    }

}


/* motor dynamics evolution function */
void maccepaPlugin::system_update(double *x, double *y, double *u) {

    /* each motor is simulated as a state-space system */
    double x_next[N];

    /* output update */
    for (int i = 0; i < M; i++) {
        y[i] = 0.0;
        for (int k = 0; k < N; k++) {
            y[i] += Cd[i][k] * x[k];
        }
    }

    /* state update */
    for (int i = 0; i < N; i++) {
        x_next[i] = 0.0;
        for (int j = 0; j < N; j++) {
            x_next[i] += Ad[i][j] * x[j];
        }
        for(int j = 0; j < U; j++) {
            x_next[i] += Bd[i][j] * u[j];
        }
    }

    for (int i = 0; i < N; i++) {
        x[i] = x_next[i];
    }

}


/*
 * CALCULATE_SPRING_FORCE calculates spring extension based on MACCEPA 2.0 geometry
 *
 * output variables:
 * FORCE - spring force (sign of force coincides with deflection angle force)
 * FORCE_ARM - arm of FORCE acting on output shaft (or on camdisk) [>=0]
 */
double maccepaPlugin::calculate_spring_force(double motor1_angle, double motor2_angle, double output_shaft_angle,
        double &force_arm) {

    double deflection_angle = motor1_angle - output_shaft_angle;

    /* limit of deflection angle */
    if (deflection_angle >= M_PI/4) {
        deflection_angle = M_PI/4;
    } else if (deflection_angle <= -M_PI/4) {
        deflection_angle = -M_PI/4;
    };

    double abs_deflection_angle = std::abs(deflection_angle);
    int sign_deflection_angle = sgn(deflection_angle);

    /* convert motor 2 angle into linear pretension */
    double pretension = motor2_angle * this->R_pretension;

    /* look for formulas in paper */
    double I = this->C * cos(abs_deflection_angle) - this->B;
    double H = this->C * sin(abs_deflection_angle) - this->R;
    double E = sqrt(I*I - this->R*this->R + H*H);
    double theta = atan(E/R);
    double gamma = atan(H/I);
    double lambda = M_PI/2 - theta + gamma;
    double kappa = M_PI/2 - theta + gamma - abs_deflection_angle;
    double force = this->k * (this->B - this->C + pretension + E + this->R*lambda) * sign_deflection_angle;

    force_arm = std::abs(this->C * sin(kappa));

    return force;
}


void maccepaPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {
    msg_test.data.resize(4);

    /* deflection angle between:
     * motor 1 output angle AND link output shaft angle */
    double deflection_angle = 0.0;

    /* discrete PID parameters */
    double P = Kp;
    double I = Ki;
    double D = Kd;

    /* motor 1 error at previous time step */
    double e1;
    /* motor 2 error at previous time step */
    double e2;

    /* linear spring torque */
    double spring_torque = 0.0;

    /* output shaft angle */
    this->output_shaft_angle = this->joint->Position();
    
    /* q1_ref e q2_ref are reference values for motor 1 and motor 2, respectively */
    motor_ref_limit(this->q1_ref, this->q2_ref);


    /***************************************************************/
    /***                   MOTOR 1 DYNAMICS                        */
    /***************************************************************/
    /* calculate motor 1 angle position at current time moment */
    /* error on previous time moment */
    e1 = this->q1_ref_prev - this->q1_prev;

    /* first input: PID generated motor torque based on previous deviation from reference position */
    u1_k[0] = P * e1 + D * (e1 - this->e1_prev) / Ts;
    /* limits control input */
    motor_torque_limit(u1_k[0]);

    /* second input: external spring generated torque from previous step */
    u1_k[1] = this->torque1_external;

    /* output and state system update */
    system_update(x1_k, y1_k, u1_k);
    this->q1 = y1_k[0];

    /***************************************************************/
    /***                   MOTOR 2 DYNAMICS                        */
    /***************************************************************/

    /* calculate motor 2 angle position at current time step*/
    /* error on previous time moment */
    e2 = this->q2_ref_prev - this->q2_prev;

    /* first input: PID generated motor torque based on previous deviation from reference position */
    u2_k[0] = P * e2 + D * (e2 - this->e2_prev) / Ts;
    /* limits control input */
    motor_torque_limit(u2_k[0]);

    /* second input: external spring generated torque from previous step */
    u2_k[1] = this->torque2_external;

    /* output and state system update */
    system_update(x2_k, y2_k, u2_k);
    this->q2 = y2_k[0];

    /***************************************************************/
    /***           OUTPUT SHAFT TORQUE COMPUTATION                 */
    /***************************************************************/

    /* signed angle between motor 1 output shaft position and link output shaft position */
    deflection_angle = this->q1 - this->output_shaft_angle;

    /* non linear torque computation (actually force and its arm) */
    double spring_force;
    double spring_force_arm;
    spring_force = calculate_spring_force(this->q1, this->q2, this->output_shaft_angle, spring_force_arm);

//    ROS_INFO_THROTTLE(0.1, "%s: %f %f %f %f %f %f %f", this->joint_name.c_str(), this->q1, this->q1_ref, this->q2, this->q2_ref, this->output_shaft_angle, spring_force, spring_force_arm);

    double viscous_friction_torque = - 1.0 * (this->output_shaft_angle - this->output_shaft_angle_prev) / Ts;
    /* output shaft torque */
    this->output_shaft_torque = spring_force * spring_force_arm + viscous_friction_torque;

    /* GAZEBO UPDATE */
    /* output shaft torque update */
    this->joint->SetForce(0, this->output_shaft_torque);
    /* motor 1 position visible indicator update */
    this->joint_motor1->SetPosition(0, this->q1);

    /* publish data to test topic */
    msg_test.data[0] = this->output_shaft_angle;
    msg_test.data[1] = spring_force;
    msg_test.data[2] = spring_force_arm;
    msg_test.data[3] = this->output_shaft_torque;
    test_pub.publish(msg_test);

    /***************************************************************/
    /***           UPDATE VARIABLES FOR THE NEXT STEP              */
    /***************************************************************/

    this->e1_prev = e1;
    this->q1_prev = this->q1;
    this->q1_ref_prev = this->q1_ref;
    this->torque1_external = - spring_force * spring_force_arm;


    this->e2_prev = e2;
    this->q2_prev = this->q2;
    this->q2_ref_prev = this->q2_ref;
    this->torque2_external = - spring_force * R_pretension;

    this->output_shaft_angle_prev = this->output_shaft_angle;
}
