#include <baker_plugin.h>
#include <boost/bind.hpp>

//###############################################################

using namespace gazebo;
using namespace std;

void bakerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    int argc = 0;
    char **argv;
    ros::init(argc, argv, "baker_plugin");

    this->model = _parent;

    /* parse plugin parameters */
    this->joint_name = _sdf->GetElement("joint")->Get<string>();
    this->topic_name = _sdf->GetElement("topic")->Get<string>();
    this->angle = _sdf->GetElement("angle")->Get<double>();

    /* bind joints */
    this->joint = this->model->GetJoint(joint_name);

    pub = n.advertise<std_msgs::Float64>(this->topic_name, 1000);

    /* bind onUpdate event */
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&bakerPlugin::OnUpdate, this, _1));

    ROS_INFO("[BAKER_PLUGIN] for \"%s\" joint loaded.", this->joint_name.c_str());
}

void bakerPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {
    double pitch = angle
            - this->joint->GetParent()->GetParentJoints().at(0)->Position()
            - this->joint->GetParent()->GetParentJoints().at(0)->GetParent()->GetParentJoints().at(0)->Position();

    std_msgs::Float64 msg;
    msg.data = pitch;
    pub.publish(msg);
}
