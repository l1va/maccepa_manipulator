#ifndef baker
#define baker

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>

#include "std_msgs/Float64.h"

namespace gazebo {

    class bakerPlugin : public ModelPlugin {

    public:

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:

        /* Name of joint to control */
        std::string joint_name;
        /* Topic which is used to publish commands for the joint */
        std::string topic_name;
        /* Desired absolute angle to hold */
        double angle;

        /* Pointer to the model */
        physics::ModelPtr model;

        /* Pointer to output shaft joint */
        physics::JointPtr joint;

        /* Variables for Gazebo topics  */
        /* Node variable */
        transport::NodePtr node;
        ros::NodeHandle n;

        /* Publisher for control commands*/
        ros::Publisher pub;

        /* Pointer to the update event connection */
        event::ConnectionPtr updateConnection;

    };

    /* Register this plugin with the simulator */
    GZ_REGISTER_MODEL_PLUGIN(bakerPlugin)

}


#endif


