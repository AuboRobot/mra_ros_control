#include "mainwindow.h"
#include <QApplication>
#include <pthread.h>
#include <mra_core_msgs/JointCommand.h>
#include <mra_basic/config.h>
#include <sensor_msgs/JointState.h>

std::vector<double> joints_init;
bool get_joints_init_position;
extern bool start_sending_joint_command;

void chatterCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    for(int i=0; i<mra_basic_config::jointID.size(); i++) {
        joints_init[i] = msg->position[i];
    }
    get_joints_init_position = true;
}


pthread_t tid;
extern std::vector<double> joints;

struct arg_holder
{
    int argc;
    char **argv;
};

arg_holder *arg_struct;

void *thread_caller(void *arg)
{
    ros::init(arg_struct->argc, arg_struct->argv, "control_panel");
    ros::NodeHandle nh;
    ros::Publisher command_pub =
            nh.advertise<mra_core_msgs::JointCommand> (JOINT_COMMAND_TOPIC, 1000);
    ros::Rate loop_rate(CONTROL_RATE);//Hz

    joints_init.resize(mra_basic_config::jointID.size());

    mra_core_msgs::JointCommand joint_command;
    joint_command.mode = mra_core_msgs::JointCommand::POSITION_MODE;
    joint_command.names = mra_basic_config::joint_names;

    ros::Subscriber sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &chatterCallback);
    get_joints_init_position = false;


    while (start_sending_joint_command == false) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(ros::ok())
    {
        ros::spinOnce();
        joint_command.command = joints;
        command_pub.publish(joint_command);

        loop_rate.sleep();
    }

    return 0;
}

int main(int argc, char *argv[])
{
    arg_struct = (arg_holder *)malloc(sizeof(struct arg_holder));
    arg_struct->argc = argc;
    arg_struct->argv = argv;

    int err = pthread_create(&tid, NULL, thread_caller, NULL);


    QApplication a(argc, argv);
    MainWindow w;
    w.show();


    return a.exec();
}
