#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <math.h>
#include <mra_basic/config.h>
#include <vector>
#include <QTime>

void delaymsec(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);

    while( QTime::currentTime() < dieTime )

        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

std::vector<double> joints;
extern std::vector<double> joints_init;
extern bool get_joints_init_position;
bool start_sending_joint_command;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_step(0.1),
    m_speed(50.0)
{
    ui->setupUi(this);

    start_sending_joint_command = false;

    joints.resize(mra_basic_config::jointID.size());
    while(get_joints_init_position == false) {
        delaymsec(500);
    }
    joints = joints_init;
    ui->lb_joint1->setText(QString::number(joints[0]*180.0/M_PI, 'f', 6));
    ui->lb_joint2->setText(QString::number(joints[1]*180.0/M_PI, 'f', 6));
    ui->lb_joint3->setText(QString::number(joints[2]*180.0/M_PI, 'f', 6));
    ui->lb_joint4->setText(QString::number(joints[3]*180.0/M_PI, 'f', 6));
    ui->lb_joint5->setText(QString::number(joints[4]*180.0/M_PI, 'f', 6));
    ui->lb_joint6->setText(QString::number(joints[5]*180.0/M_PI, 'f', 6));
    ui->lb_joint7->setText(QString::number(joints[6]*180.0/M_PI, 'f', 6));

    start_sending_joint_command = true;

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pbn_joint1Left_pressed()
{
    joints[0] = joints[0] - m_step * m_speed/100 < -M_PI ?
                joints[0] :
            joints[0] - m_step  * m_speed/100;

    ui->lb_joint1->setText(QString::number(joints[0]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint1Right_pressed()
{
    joints[0] = joints[0] + m_step * m_speed/100 > M_PI ?
                joints[0] :
            joints[0] + m_step * m_speed/100;

    ui->lb_joint1->setText(QString::number(joints[0]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint2Left_pressed()
{
    joints[1] = joints[1] - m_step * m_speed/100 < -M_PI ?
                joints[1] :
            joints[1] - m_step * m_speed/100;

    ui->lb_joint2->setText(QString::number(joints[1]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint2Right_pressed()
{
    joints[1] = joints[1] + m_step * m_speed/100 > M_PI ?
                joints[1] :
            joints[1] + m_step * m_speed/100;

    ui->lb_joint2->setText(QString::number(joints[1]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint3Left_pressed()
{
    joints[2] = joints[2] - m_step * m_speed/100 < -M_PI ?
                joints[2] :
            joints[2] - m_step * m_speed/100;
    ui->lb_joint3->setText(QString::number(joints[2]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint3Right_pressed()
{
    joints[2] = joints[2] + m_step * m_speed/100 > M_PI ?
                joints[2] :
            joints[2] + m_step * m_speed/100;

    ui->lb_joint3->setText(QString::number(joints[2]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint4Left_pressed()
{
    joints[3] = joints[3] - m_step * m_speed/100 < -M_PI ?
                joints[3] :
            joints[3] - m_step * m_speed/100;

    ui->lb_joint4->setText(QString::number(joints[3]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint4Right_pressed()
{
    joints[3] = joints[3] + m_step * m_speed/100 > M_PI ?
                joints[3] :
            joints[3] + m_step * m_speed/100;

    ui->lb_joint4->setText(QString::number(joints[3]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint5Left_pressed()
{
    joints[4] = joints[4] - m_step * m_speed/100 < -M_PI ?
                joints[4] :
            joints[4] - m_step * m_speed/100;

    ui->lb_joint5->setText(QString::number(joints[4]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint5Right_pressed()
{
    joints[4] = joints[4] + m_step * m_speed/100 > M_PI ?
                joints[4] :
            joints[4] + m_step * m_speed/100;

    ui->lb_joint5->setText(QString::number(joints[4]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint6Left_pressed()
{
    joints[5] = joints[5] - m_step * m_speed/100 < -M_PI ?
                joints[5] :
            joints[5] - m_step * m_speed/100;

    ui->lb_joint6->setText(QString::number(joints[5]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint6Right_pressed()
{
    joints[5] = joints[5] + m_step * m_speed/100 > M_PI ?
                joints[5] :
            joints[5] + m_step * m_speed/100;

    ui->lb_joint6->setText(QString::number(joints[5]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint7Left_pressed()
{
    joints[6] = joints[6] - m_step * m_speed/100 < -M_PI ?
                joints[6] :
            joints[6] - m_step * m_speed/100;

    ui->lb_joint7->setText(QString::number(joints[6]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint7Right_pressed()
{
    joints[6] = joints[6] + m_step * m_speed/100 > M_PI ?
                joints[6] :
            joints[6] + m_step * m_speed/100;

    ui->lb_joint7->setText(QString::number(joints[6]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pushButton_clicked()
{
    for (int i=0; i<joints.size(); i++) {
        joints[i] = 0.0;
    }
    ui->lb_joint1->setText(QString::number(joints[0], 'f', 6));
    ui->lb_joint2->setText(QString::number(joints[1], 'f', 6));
    ui->lb_joint3->setText(QString::number(joints[2], 'f', 6));
    ui->lb_joint4->setText(QString::number(joints[3], 'f', 6));
    ui->lb_joint5->setText(QString::number(joints[4], 'f', 6));
    ui->lb_joint6->setText(QString::number(joints[5], 'f', 6));
    ui->lb_joint7->setText(QString::number(joints[6], 'f', 6));
}
