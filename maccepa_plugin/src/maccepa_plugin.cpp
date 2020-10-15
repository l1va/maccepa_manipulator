#include <maccepa_plugin.h>
#include <boost/bind.hpp>

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

    /* parse plugin parameters */
    if(_sdf->HasElement("verbose")) {
        this->verbose = _sdf->GetElement("verbose")->Get<bool>();
    }
    std::string joint_name = _sdf->GetElement("joint")->Get<string>();
    std::string cardioid_motor_joint_name = _sdf->GetElement("cardioid_motor_joint")->Get<string>();
    std::string cardioid_motor_command_topic = _sdf->GetElement("cardioid_motor_command_topic")->Get<string>();
    std::string pretension_motor_command_topic = _sdf->GetElement("pretension_motor_command_topic")->Get<string>();
    std::string motors_state_topic;
    if(_sdf->HasElement("motors_state_topic")) {
        motors_state_topic = _sdf->GetElement("motors_state_topic")->Get<string>();
    } else {
        ROS_WARN("[MACCEPA_PLUGIN] No motors state topic specified");
    }
    this->k = _sdf->GetElement("k")->Get<double>();
    this->B = _sdf->GetElement("B")->Get<double>();
    this->C = _sdf->GetElement("C")->Get<double>();
    this->R = _sdf->GetElement("R")->Get<double>();
    this->R_pretension = _sdf->GetElement("R_pretension")->Get<double>();
    if(_sdf->HasElement("initial_pretension")) {
        this->initial_pretension = _sdf->GetElement("initial_pretension")->Get<double>();
    } else {
        this->initial_pretension = 0.0;
    }


    /* bind joints */
    this->joint_name = joint_name;
    this->joint = this->model->GetJoint(joint_name);
    this->cardioid_motor_joint = this->model->GetJoint(cardioid_motor_joint_name);

    /* INITIALISATION */
    init();

    /* PID parameters in discrete domain at initialization */
    this->Kp = 686.0;
    this->Ki = 0*10.0;
    this->Kd = 12.0;

    /* create topic subscribers & publishers and bind them to callbacks*/
    cardioid_sub = n.subscribe(cardioid_motor_command_topic, 1, &maccepaPlugin::cardioid_motor_callback, this);
    pretension_sub = n.subscribe(pretension_motor_command_topic, 1, &maccepaPlugin::pretension_motor_callback, this);

    if(!motors_state_topic.empty()) {
        this->motors_pub = n.advertise<sensor_msgs::JointState>(motors_state_topic, 500);
    }

    test_pub = n.advertise<std_msgs::Float64MultiArray>("test", 500);

    /* bind onUpdate event */
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&maccepaPlugin::OnUpdate, this, _1));
    /* bind onReset event */
    this->resetConnection = event::Events::ConnectWorldReset(boost::bind(&maccepaPlugin::OnReset, this));

    /* Initialize messages */
    msg_test.data.resize(4);

    if(this->motors_pub != NULL) {
        this->motors_msg.name.resize(2);
        this->motors_msg.position.resize(2);
        this->motors_msg.effort.resize(2);
    }

    ROS_INFO("[MACCEPA_PLUGIN] for \"%s\" joint loaded.", this->joint_name.c_str());
}

void maccepaPlugin::init() {
    /* angle of cardioid motor in radians */
    this->q1 = 0.0;
    this->q1_prev = 0.0;

    /* angle of pretension motor in radians */
    this->q2 = this->initial_pretension;
    this->q2_prev = this->initial_pretension;

    /* reference angle for cardioid motor in radians */
    this->q1_ref = 0.0;
    this->q1_ref_prev = 0.0;

    /* reference angle for pretension motor in radians */
    this->q2_ref = this->initial_pretension;
    this->q2_ref_prev = this->initial_pretension;

    this->e1_prev = 0.0;
    this->e2_prev = 0.0;

    this->output_shaft_angle = 0.0;
    this->output_shaft_angle_prev = 0.0;

    /* state initialization */
    for (int i = 0; i < N; i++) {
        x1_k[i] = 0.0;
        x2_k[i] = 0.0;
    }
    x1_k[0] = this->q1;
    x2_k[0] = this->q2;

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

    updateTorques();
}


/* updates cardioid motor reference position */
void maccepaPlugin::cardioid_motor_callback(const std_msgs::Float64::ConstPtr &msg) {
    this->q1_ref = msg->data;

    if(this->verbose) {
        ROS_INFO("[MACCEPA_PLUGIN] New cardioid motor reference position: %f", this->q1_ref);
    }
}


/* updates pretension motor reference position */
void maccepaPlugin::pretension_motor_callback(const std_msgs::Float64::ConstPtr &msg) {
    this->q2_ref = msg->data;

    if(this->verbose) {
        ROS_INFO("[MACCEPA_PLUGIN] New pretension motor reference position: %f", this->q2_ref);
    }
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
 * FORCE - spring force [>=0]
 * FORCE_ARM - arm of FORCE acting on output shaft (or on camdisk)
 *            [sign of force arm coincides with deflection angle]
 */
double maccepaPlugin::calculate_spring_force(double motor1_angle, double motor2_angle, double output_shaft_angle,
        double &force_arm) {

    /* deflection angle between:
     * motor 1 output angle AND link output shaft angle */
    /* signed angle between motor 1 output shaft position and link output shaft position */
    double deflection_angle = motor1_angle - output_shaft_angle;

    /* Limit of deflection angle */
    if (deflection_angle >= M_PI/4) {
        deflection_angle = M_PI/4;
    } else if (deflection_angle <= -M_PI/4) {
        deflection_angle = -M_PI/4;
    };

    double abs_deflection_angle = std::abs(deflection_angle);
    int sign_deflection_angle = sgn(deflection_angle);

    /* Convert motor 2 angle into linear pretension */
    double pretension = motor2_angle * this->R_pretension;

    /* Look for formulas in paper */
    double I = this->C * cos(abs_deflection_angle) - this->B;
    double H = this->C * sin(abs_deflection_angle) - this->R;
    double E = sqrt(I*I - this->R*this->R + H*H);
    double theta = atan(E/R);
    double gamma = atan(H/I);
    double lambda = M_PI/2 - theta + gamma;
    double kappa = M_PI/2 - theta + gamma - abs_deflection_angle;

    /* Force is not negative */
    double force = this->k * (this->B - this->C + pretension + E + this->R*lambda);

    /* Arm of force have the same sign as deflection angle */
    force_arm = std::abs(this->C * sin(kappa)) * sign_deflection_angle;

    return force;
}

/* calculate and update torques generated by MACCEPA 2.0 geometry */
void maccepaPlugin::updateTorques() {
    /* non linear torque computation (actually force and its arm) */
    double spring_force;
    double spring_force_arm;
    spring_force = calculate_spring_force(this->q1, this->q2, this->output_shaft_angle, spring_force_arm);

    /* viscous force for damping */
    double viscous_friction_torque = - 1.0 * (this->output_shaft_angle - this->output_shaft_angle_prev) / Ts;

    /* update torques */
    this->output_shaft_torque = spring_force * spring_force_arm + viscous_friction_torque;
    this->torque1_external = - spring_force * spring_force_arm;
    this->torque2_external = spring_force * R_pretension;
}

/* update motor state on the current time step using previous state, errors and torque */
void maccepaPlugin::updateMotorState(double &e, double &q, double *x, double *y, double *u,
                                       double q_ref_prev, double q_prev, double e_prev, double torque_external) {
    /* error on previous time moment */
    double error = q_ref_prev - q_prev;

    /* first input: PID generated motor torque based on previous deviation from reference position */
    u[0] = Kp * e + Kd * (error - e_prev) / Ts;
    /* second input: external spring generated torque from previous step */
    u[1] = torque_external;

    /* limit control input */
    motor_torque_limit(u[0]);

    /* output and state system update */
    system_update(x, y, u);

    q =  y[0];
    e = error;
}

/* GAZEBO onWorldReset event callback */
void maccepaPlugin::OnReset() {
    init();
    ROS_WARN("[MACCEPA_PLUGIN] reset");
}

/* GAZEBO onWorldUpdate event callback */
void maccepaPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {

    /* get output shaft angle from gazebo */
    this->output_shaft_angle = this->joint->Position();

    /* limit motor reference values with respect to constraints */
    motor_ref_limit(this->q1_ref, this->q2_ref);

    /***************************************************************/
    /***           MOTOR DYNAMICS UPDATE                           */
    /***************************************************************/
    /* update cardioid motor state */
    updateMotorState(this->e1_prev, this->q1, x1_k, y1_k, u1_k, this->q1_ref_prev, this->q1_prev, this->e1_prev, this->torque1_external);
    /* update pretension motor state */
    updateMotorState(this->e2_prev, this->q2, x2_k, y2_k, u2_k, this->q2_ref_prev, this->q2_prev, this->e2_prev, this->torque2_external);


    /***************************************************************/
    /***           TORQUES COMPUTATION                             */
    /***************************************************************/
    updateTorques();

    /* GAZEBO UPDATE */
    /* output shaft torque update */
    this->joint->SetForce(0, this->output_shaft_torque);
    /* cardioid motor position indicator update */
    this->cardioid_motor_joint->SetPosition(0, this->q1);

    /* publish data to test topic */
    msg_test.data[0] = this->output_shaft_angle;
    msg_test.data[1] = this->q1;
    msg_test.data[2] = this->q2;
    msg_test.data[3] = this->output_shaft_torque;
    test_pub.publish(msg_test);

    /* Publish motor states*/
    if(this->motors_pub != NULL) {
        this->motors_msg.name[0] = "cardioid";
        this->motors_msg.position[0] = this->q1;
        this->motors_msg.effort[0] = this->output_shaft_torque; //DEBUG
        this->motors_msg.name[1] = "pretension";
        this->motors_msg.position[1] = this->q2;
        this->motors_msg.effort[1] = this->output_shaft_torque; //DEBUG
        this->motors_pub.publish(this->motors_msg);
    }

    /***************************************************************/
    /***           UPDATE VARIABLES FOR THE NEXT STEP              */
    /***************************************************************/
    this->q1_prev = this->q1;
    this->q1_ref_prev = this->q1_ref;

    this->q2_prev = this->q2;
    this->q2_ref_prev = this->q2_ref;

    this->output_shaft_angle_prev = this->output_shaft_angle;
}
