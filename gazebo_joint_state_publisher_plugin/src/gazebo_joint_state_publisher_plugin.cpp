#include <gazebo_joint_state_publisher_plugin.h>

//###############################################################

using namespace gazebo;
using namespace std;

void gazeboJointStatePublisherPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    int argc = 0;
    char **argv;
    ros::init(argc, argv, "gazebo_joint_state_publisher_plugin");

    this->model = _parent;

    /* parse plugin parameters */
    std::string joint_names_string = _sdf->GetElement("joints")->Get<string>();
    std::string topic_name = _sdf->GetElement("topic")->Get<string>();

    std::stringstream ss(joint_names_string);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> joint_names(begin, end);

    for(unsigned i = 0; i < joint_names.size(); i++) {
        this->joints.push_back(this->model->GetJoint(joint_names[i]));
        this->msg.name.push_back(joint_names[i]);
        this->msg.position.push_back(0.0);
        this->msg.velocity.push_back(0.0);
        this->msg.effort.push_back(0.0);
    }

    pub = n.advertise<sensor_msgs::JointState>(topic_name, 1000);

    /* bind onUpdate event */
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&gazeboJointStatePublisherPlugin::OnUpdate, this, _1));

    ROS_INFO("[GAZEBO_JOINT_STATE_PUBLISHER_PLUGIN] for \"%s\" joints loaded.", joint_names_string.c_str());
}

void gazeboJointStatePublisherPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {
    for(unsigned i = 0; i < this->joints.size(); i++) {
        this->msg.position[i] = this->joints[i]->Position();
        this->msg.velocity[i] = this->joints[i]->GetVelocity(0);
        this->msg.effort[i] = this->joints[i]->GetForce(0);
    }
    pub.publish(this->msg);
}
