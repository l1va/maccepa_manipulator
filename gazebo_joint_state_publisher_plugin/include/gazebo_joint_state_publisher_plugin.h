#ifndef gazebo_joint_state_publisher
#define gazebo_joint_state_publisher

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <stdio.h>

#include <ros/ros.h>
#include <ros/console.h>
#include "sensor_msgs/JointState.h"

namespace gazebo {

    class gazeboJointStatePublisherPlugin : public ModelPlugin {

    public:

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:

        /* Names of joints to publish state */
//        std::vector<std::string> joint_names;
        /* Topic which is used to publish commands for the joint */
        std::string topic_name;

        std::vector<physics::JointPtr> joints;


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

        /* */
        sensor_msgs::JointState msg;

        /* Pointer to the update event connection */
        event::ConnectionPtr updateConnection;

    };

    /* Register this plugin with the simulator */
    GZ_REGISTER_MODEL_PLUGIN(gazeboJointStatePublisherPlugin)

}


#endif


