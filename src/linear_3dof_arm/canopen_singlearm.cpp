#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseArray.h"

#include <string>
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <linear_3dof_arm/controlcan.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"

#include <errno.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>


#include <signal.h>
volatile sig_atomic_t sig_int_received = 0;
void sig_int_handler(int sig){
  sig_int_received = 1;
}

using namespace std;
ros::Publisher arm_cur_pos;
ros::Publisher read_pub;
ros::Publisher read2_pub;
ros::Publisher read3_pub;

geometry_msgs::PoseArray arm_pos;
std_msgs::String result;

int former_arm_x = 0;
int former_arm_y = 0;
int former_arm_z = 0;
int acceleration = 70;

bool read_current_pos = false;
bool read_current_status;
bool acc_rep_x_h = false;
bool acc_rep_y_h = false;
bool acc_rep_z_h = false;
bool acc_rep_x_l = false;
bool acc_rep_y_l = false;
bool acc_rep_z_l = false;
float x_axis_ratio = 172; // mm/r
float y_axis_ratio = 100;
float z_axis_ratio = 20;
float station_axis_ratio = 70;

bool reach_status = false;
bool all_arms_reached = false;
bool zeroing_mark_arm1 = false;
bool zeroing_mark_arm2 = false;
bool zeroing_mark_station = false;

bool pause_resume_arm1 = false;
bool pause_resume_arm2 = false;
bool pause_resume_station = false;

void send_motor(int motor_id, int data_length, int byte1, int byte2, int byte3, int byte4, int byte5, int byte6, int byte7, int byte8) {
    VCI_CAN_OBJ send[1];
    send[0].ID = motor_id;
    send[0].SendType = 0; //0 is normal
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0; //0 means standard ID (11 bits)
    send[0].DataLen = data_length;

    if (data_length == 8) {
        send[0].Data[0] = byte1;
        send[0].Data[1] = byte2;
        send[0].Data[2] = byte3;
        send[0].Data[3] = byte4;
        send[0].Data[4] = byte5;
        send[0].Data[5] = byte6;
        send[0].Data[6] = byte7;
        send[0].Data[7] = byte8;
    } else if (data_length == 2) {
        send[0].ID = 0;
        send[0].Data[0] = 1;
        send[0].Data[1] = motor_id;
    }
    //cout<<send[0].ID<<send[0].Data[0]<<send[0].Data[1]<<send[0].Data[2]<<send[0].Data[3]<<send[0].Data[4]<<send[0].Data[5]<<send[0].Data[6]<<send[0].Data[7]<<endl;
    if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) != 1) {
        ROS_ERROR_STREAM("Failed sending data");
    }
}

float read_motor_pos(int motor_id) {
    float position_mm = 0;
    VCI_ClearBuffer(VCI_USBCAN2, 0, 0);

    while (read_current_pos == false) { // make sure it can read position
        send_motor(motor_id, 8, 67, 100, 96, 0, 0, 0, 0, 0); //read current position

        int reclen = 0; //reclen is the number of msgs in the buffer
        VCI_CAN_OBJ rec[2000]; //receive buffer, 3000 is the best
        int i, j;
        int ind = 0;

        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 2000, 100)) > 0) //call receive function, if data are available, continue to process
        {
            // printf("reclen = %d\n",reclen);

            for (j = 0; j < reclen; j++) {
                if (rec[j].Data[0] == 67 && rec[j].Data[1] == 100 && rec[j].Data[2] == 96) {
                    long int position_steps = rec[j].Data[4] + rec[j].Data[5] * 256 + rec[j].Data[6] * 65536; //ingnore data[7], out of range

                    if (motor_id == 1537) {
                        position_mm = x_axis_ratio * (position_steps - 16777215) / 4000.0;

                    } else if (motor_id == 1540) {
                        position_mm = x_axis_ratio * position_steps / 4000.0;
                    } else if (motor_id == 1538 || motor_id == 1541) {
                        position_mm = y_axis_ratio * position_steps / 4000.0;
                    } else if (motor_id == 1539 || motor_id == 1542) {
                        position_mm = z_axis_ratio * position_steps / 4000.0;
                    }
                    read_current_pos = true;
                }
            }
        }
    }
    read_current_pos = false;
    return position_mm;
}

void read_cur_pos() {
    geometry_msgs::Pose arm_pos_single;
    arm_pos_single.position.x = read_motor_pos(1540);
    arm_pos_single.position.y = read_motor_pos(1541);
    arm_pos_single.position.z = read_motor_pos(1542);
    arm_pos.poses.push_back(arm_pos_single);
    // cout<<arm_pos_single.position.x<<' '<<arm_pos_single.position.y<<' '<<arm_pos_single.position.z<<endl;
    arm_pos.header.frame_id = "/linear_3dof_arm_home";
    arm_cur_pos.publish(arm_pos);
    arm_pos.poses.clear();
}

float read_motor_status(int motor_id) {
    float position_mm = 0;
    VCI_ClearBuffer(VCI_USBCAN2, 0, 0);

    while (read_current_status == false) { // make sure it has read required info after sending command
        send_motor(motor_id, 8, 75, 65, 96, 0, 0, 0, 0, 0); //read current status
        int reclen = 0; //reclen is the number of msgs in the buffer
        VCI_CAN_OBJ rec[2000]; //receive buffer, 3000 is the best
        int j;
        int ind = 0;
        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 2000, 100)) > 0) //call receive function, if data are available, continue to process
        {

            for (j = 0; j < reclen; j++) {

                if (rec[j].DataLen > 2) {
                    if (rec[j].Data[0] == 75 && rec[j].Data[1] == 65 && rec[j].Data[2] == 96) {
                        read_current_status = true;
                        if ((rec[j].Data[5] >> 2) & 0x1) {
                            reach_status = true;
                        } else {
                            reach_status = false;
                        }
                    }
                }
            }

        }
    }
    read_current_status = false;
    usleep(5000); // 10ms

    return reach_status;
}

void read_cur_status() {
    if (!all_arms_reached && read_motor_status(1540) && read_motor_status(1541) && read_motor_status(1542)) {
        all_arms_reached = true; // only publish the message once
        read2_pub.publish(result); //publish reach target position msgs
    }
}

void arm_position_send(int motor_id, int hig_byt3_arm, int hig_byt2_arm, int hig_byt_arm, int low_byt_arm, int hig_byt_arm_speed, int low_byt_arm_speed) { //calibrate arm
    //printf("motor_id = %d\fhig_byt3_arm = %d\fhig_byt2_arm %d\fhig_byt_arm = %d\flow_byt_arm = %d\n",motor_id, hig_byt3_arm, hig_byt2_arm, hig_byt_arm, low_byt_arm);
    send_motor(motor_id, 8, 35, 129, 96, 0, low_byt_arm_speed, hig_byt_arm_speed, 0, 0); //set speed to 1.0 rps
    usleep(10000); // 10ms
    send_motor(motor_id, 8, 35, 122, 96, 0, low_byt_arm, hig_byt_arm, hig_byt2_arm, hig_byt3_arm); //set targte steps
    usleep(10000); // 10ms
    send_motor(motor_id, 8, 43, 64, 96, 0, 48 + 15, 0, 0, 0); //set new position (start running)  15+16 completed and excute the next one. send_motor(motor_id, 8, 43, 64, 96, 0, 15+48, 2, 0, 0); //move to new position without stop
    usleep(10000);
    send_motor(motor_id, 8, 43, 64, 96, 0, 15, 0, 0, 0); //clear mark send_motor(motor_id, 8, 43, 64, 96, 0, 15, 2, 0, 0);
    usleep(10000);
}

void zeroing(int motor_id, int hig_byt_arm_speed, int low_byt_arm_speed) {
    send_motor(motor_id, 8, 47, 96, 96, 0, 6, 0, 0, 0); //set to zeroing mode
    usleep(10000); // 5ms
    if (motor_id == 1537 || motor_id == 1543) {
        send_motor(motor_id, 8, 47, 152, 96, 0, 5, 0, 0, 0); //select zeroing mode 5 cw
        usleep(10000); // 5ms
    } else {
        send_motor(motor_id, 8, 47, 152, 96, 0, 6, 0, 0, 0); //select zeroing mode 6 ccw
        usleep(10000); // 5ms
    }
    send_motor(motor_id, 8, 43, 154, 96, 0, acceleration, 0, 0, 0); //set acceleration to 100 rps/s
    usleep(10000); // 5ms
    send_motor(motor_id, 8, 35, 153, 96, 1, low_byt_arm_speed, hig_byt_arm_speed, 0, 0); //set speed to 0.2 rps
    usleep(10000);
    send_motor(motor_id, 8, 35, 153, 96, 2, 5, 0, 0, 0); //Set searching switch sensors speed to 0.5rps
    usleep(10000); // 5ms
    send_motor(motor_id, 8, 43, 64, 96, 0, 31, 0, 0, 0); //set new position (start running)
    usleep(10000); // 5ms

}

void arm_position(int motorx_id, int motory_id, int motorz_id, float arm_x, float arm_y, float arm_z, float arm_speed, int pausearm) {
    if (abs(former_arm_x - arm_x) > 20 && acc_rep_x_h == false) {
        send_motor(motorx_id, 8, 43, 131, 96, 0, 30, 0, 0, 0); //set acceleration to 100 rps/s
        usleep(5000); // 10ms
        send_motor(motorx_id, 8, 43, 132, 96, 0, 30, 0, 0, 0); //set Deceleration to 100 rps/s
        usleep(5000); // 10ms
        acc_rep_x_h = true;
        acc_rep_x_l = false;
    } else if (abs(former_arm_x - arm_x) <= 20 && acc_rep_x_l == false) {
        send_motor(motorx_id, 8, 43, 131, 96, 0, 10, 0, 0, 0); //set acceleration to 100 rps/s
        usleep(5000); // 10ms
        send_motor(motorx_id, 8, 43, 132, 96, 0, 10, 0, 0, 0); //set Deceleration to 100 rps/s
        usleep(5000); // 10ms
        acc_rep_x_l = true;
        acc_rep_x_h = false;
    }

    if (abs(former_arm_y - arm_y) > 20 && acc_rep_y_h == false) {
        send_motor(motory_id, 8, 43, 131, 96, 0, 180, 0, 0, 0); //set acceleration to 100 rps/s
        usleep(5000); // 10ms
        send_motor(motory_id, 8, 43, 132, 96, 0, 180, 0, 0, 0); //set Deceleration to 100 rps/s
        usleep(5000); // 10ms
        acc_rep_y_h = true;
        acc_rep_y_l = false;
    } else if (abs(former_arm_y - arm_y) <= 20 && acc_rep_y_l == false) {
        send_motor(motory_id, 8, 43, 131, 96, 0, 80, 0, 0, 0); //set acceleration to 100 rps/s
        usleep(5000); // 10ms
        send_motor(motory_id, 8, 43, 132, 96, 0, 80, 0, 0, 0); //set Deceleration to 100 rps/s
        usleep(5000); // 10ms
        acc_rep_y_l = true;
        acc_rep_y_h = false;
    }

    if (abs(former_arm_z - arm_z) > 20 && acc_rep_z_h == false) {
        send_motor(motorz_id, 8, 43, 131, 96, 0, 232, 1, 0, 0); //set acceleration to 100 rps/s
        usleep(5000); // 10ms
        send_motor(motorz_id, 8, 43, 132, 96, 0, 232, 1, 0, 0); //set Deceleration to 100 rps/s
        usleep(5000); // 10ms
        acc_rep_z_h = true;
        acc_rep_z_l = false;
    } else if (abs(former_arm_z - arm_z) <= 20 && acc_rep_z_l == false) {
        send_motor(motorz_id, 8, 43, 131, 96, 0, 232, 0, 0, 0); //set acceleration to 100 rps/s
        usleep(5000); // 10ms
        send_motor(motorz_id, 8, 43, 132, 96, 0, 232, 0, 0, 0); //set Deceleration to 100 rps/s
        usleep(5000); // 10ms
        acc_rep_z_l = true;
        acc_rep_z_h = false;
    }
    former_arm_x = arm_x;
    former_arm_y = arm_y;
    former_arm_z = arm_z;

    unsigned long int arm_x_steps = 0, arm_y_steps = 0, arm_z_steps = 0;
    if (pausearm == 0 || pausearm == 6) {
        if (arm_x > 0) {
            arm_x_steps = 4000 * arm_x / x_axis_ratio;
        } else {
            arm_x_steps = 4294967295 + 4000.0 * arm_x / x_axis_ratio;
        }
        int armx_speed_rps = 10 * arm_speed / x_axis_ratio; // 10 rps
        arm_y > 0 ? arm_y_steps = 4000 * arm_y / y_axis_ratio : arm_y_steps = 4294967295 + 4000.0 * arm_y / y_axis_ratio;
        //int arm_y_steps=4000*arm_y/y_axis_ratio;
        int army_speed_rps = 10 * arm_speed / y_axis_ratio; // 10 rps
        arm_z > 0 ? arm_z_steps = 4000 * arm_z / z_axis_ratio : arm_z_steps = 4294967295 + 4000.0 * arm_z / z_axis_ratio;

        //int arm_z_steps=4000*arm_z/z_axis_ratio;
        int armz_speed_rps = pausearm == 0 ? 10 * arm_speed / z_axis_ratio : 0.3 * 10 * arm_speed / z_axis_ratio;

        //printf("arm1_x_steps = %d\tarm1_x = %f\n",arm_x_steps,arm_x);
        int hig_byt3_arm_x = arm_x_steps / 16777216;
        int hig_byt2_arm_x = (arm_x_steps % 16777216) / 65536;
        int hig_byt_arm_x = (arm_x_steps % 65536) / 256;
        int low_byt_arm_x = arm_x_steps % 256;

        int hig_byt3_arm_y = arm_y_steps / 16777216;
        int hig_byt2_arm_y = (arm_y_steps % 16777216) / 65536;
        int hig_byt_arm_y = (arm_y_steps % 65536) / 256;
        int low_byt_arm_y = arm_y_steps % 256;

        int hig_byt3_arm_z = arm_z_steps / 16777216;
        int hig_byt2_arm_z = (arm_z_steps % 16777216) / 65536;
        int hig_byt_arm_z = (arm_z_steps % 65536) / 256;
        int low_byt_arm_z = arm_z_steps % 256;

        int hig_byt_armx_speed = armx_speed_rps / 256;
        int low_byt_armx_speed = armx_speed_rps % 256;
        int hig_byt_army_speed = army_speed_rps / 256;
        int low_byt_army_speed = army_speed_rps % 256;
        int hig_byt_armz_speed = armz_speed_rps / 256;
        int low_byt_armz_speed = armz_speed_rps % 256;

        //printf("hig_byt_arm_x = %d\tlow_byt_arm_x = %d\n",hig_byt_arm_x,low_byt_arm_x);
        //printf("hig_byt_arm_speed = %d\tlow_byt_arm_speed = %d\n",hig_byt_arm_speed,low_byt_arm_speed);
        arm_position_send(motorx_id, hig_byt3_arm_x, hig_byt2_arm_x, hig_byt_arm_x, low_byt_arm_x, hig_byt_armx_speed, low_byt_armx_speed);
        arm_position_send(motory_id, hig_byt3_arm_y, hig_byt2_arm_y, hig_byt_arm_y, low_byt_arm_y, hig_byt_army_speed, low_byt_army_speed);
        arm_position_send(motorz_id, hig_byt3_arm_z, hig_byt2_arm_z, hig_byt_arm_z, low_byt_arm_z, hig_byt_armz_speed, low_byt_armz_speed);
    } else if (pausearm == 1) //pause
    {
        send_motor(motorx_id, 8, 43, 64, 96, 0, 31, 1, 0, 0); //pause motor
        send_motor(motory_id, 8, 43, 64, 96, 0, 31, 1, 0, 0); //pause motor
        send_motor(motorz_id, 8, 43, 64, 96, 0, 31, 1, 0, 0); //pause motor

    } else if (pausearm == 2) //resume
    {
        send_motor(motorx_id, 8, 43, 64, 96, 0, 31, 0, 0, 0); //resume motor
        send_motor(motory_id, 8, 43, 64, 96, 0, 31, 0, 0, 0); //resume motor
        send_motor(motorz_id, 8, 43, 64, 96, 0, 31, 0, 0, 0); //resume motor
        if (motorx_id == 1537 && motory_id == 1538 && motorz_id == 1539) {
            pause_resume_arm1 = true;
        } else if (motorx_id == 1540 && motory_id == 1541 && motorz_id == 1542) {
            pause_resume_arm2 = true;
        }
    } else if (pausearm == 3) //unblock motors
    {
        send_motor(motorx_id, 8, 43, 64, 96, 0, 3, 0, 0, 0);
        usleep(5000); // 5ms
        send_motor(motory_id, 8, 43, 64, 96, 0, 3, 0, 0, 0);
        usleep(5000); // 5ms
        send_motor(motorz_id, 8, 43, 64, 96, 0, 3, 0, 0, 0);
        usleep(5000); // 5ms
    } else if (pausearm == 4) //zeroing motors
    {
        int armx_speed_rps = 10 * arm_speed / x_axis_ratio; // 10 rps
        int army_speed_rps = 10 * arm_speed / y_axis_ratio; // 10 rps
        int armz_speed_rps = 10 * arm_speed / z_axis_ratio; // 10 rps
        int hig_byt_armx_speed = armx_speed_rps / 256;
        int low_byt_armx_speed = armx_speed_rps % 256;
        int hig_byt_army_speed = army_speed_rps / 256;
        int low_byt_army_speed = army_speed_rps % 256;
        int hig_byt_armz_speed = armz_speed_rps / 256;
        int low_byt_armz_speed = armz_speed_rps % 256;

        zeroing(motory_id, hig_byt_army_speed, low_byt_army_speed);
        zeroing(motorz_id, hig_byt_armz_speed, low_byt_armz_speed);
        if (motorx_id == 1540 && motory_id == 1541 && motorz_id == 1542) {
            sleep(2);
            zeroing_mark_arm2 = true;

        } else if (motorx_id == 1537 && motory_id == 1538 && motorz_id == 1539) {
            zeroing_mark_arm1 = true;
        }
        zeroing(motorx_id, hig_byt_armx_speed, low_byt_armx_speed);
    } else if (pausearm == 5) //reset motors
    {
        send_motor(motorx_id, 8, 43, 64, 96, 0, 1, 0, 0, 0); //reset
        usleep(10000); // 10ms
        send_motor(motorx_id, 8, 43, 64, 96, 0, 15, 0, 0, 0); //enable motor
        usleep(10000); // 10ms
        send_motor(motory_id, 8, 43, 64, 96, 0, 1, 0, 0, 0); //reset
        usleep(10000); // 10ms
        send_motor(motory_id, 8, 43, 64, 96, 0, 15, 0, 0, 0); //enable motor
        usleep(10000); // 10ms
        send_motor(motorz_id, 8, 43, 64, 96, 0, 1, 0, 0, 0); //reset
        usleep(10000); // 10ms
        send_motor(motorz_id, 8, 43, 64, 96, 0, 15, 0, 0, 0); //enable motor
        usleep(10000); // 10ms
        former_arm_x = 0;
        former_arm_y = 0;
        former_arm_z = 0;
    }
}

void arm2_callback(const std_msgs::Float32MultiArray::ConstPtr & msag2) {
    all_arms_reached = false;

    if (zeroing_mark_arm2 == true || pause_resume_arm2 == true) {
        for (int i = 1540; i <= 1542; i++) //ID from 601 to 606
        {
            send_motor(i, 8, 43, 64, 96, 0, 15, 0, 0, 0); //clear mark
            usleep(5000);
            send_motor(i, 8, 47, 96, 96, 0, 1, 0, 0, 0); //set to position mode
            usleep(5000); // 10ms
        }
        zeroing_mark_arm2 = false;
        pause_resume_arm2 = false;

    }
    arm_position(1540, 1541, 1542, msag2->data[0], msag2->data[1], msag2->data[2], msag2->data[3], msag2->data[4]); //x pos, y pos, z pos, speed

}

void throw_error(string message) {
    throw runtime_error(message);
}

int main(int argc, char ** argv) {
    // Signal to catch ctrl+c event POSIX only
    signal(SIGINT, sig_int_handler);

    const char *filename = "/dev/usb2can";
    int fd;
    int rc;

    fd = open(filename, O_WRONLY);
    if (fd < 0) {
        throw_error("Error opening output file");
    }

    ROS_INFO_STREAM("Resetting USB device /dev/usb2can");
    rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0) {
        throw_error("Error in ioctl");
    }
    ROS_INFO_STREAM("Reset successful");

    close(fd);

    int return_status = 0;

    try{
        ros::init(argc, argv, "listener");

        ros::NodeHandle nh;
        ros::Subscriber write2_sub = nh.subscribe("/linear_3dof_arm/arm/raw/arm2position", 1000, arm2_callback);

        arm_cur_pos = nh.advertise<geometry_msgs::PoseArray>("/linear_3dof_arm/arm/raw/cur_pos", 1);
        read2_pub = nh.advertise<std_msgs::String > ("/linear_3dof_arm/arm/raw/arm2reached", 1000);

        result.data = "Arm reached target position";

        if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1) //open device
        {
            ROS_INFO_STREAM("CAN Open initialized");
        } else {
            throw_error("Unable to open device or already opened");
        }

        // Initialisation
        VCI_INIT_CONFIG config;
        config.AccCode = 0;
        config.AccMask = 0xFFFFFFFF;
        config.Filter = 1; //receive all data
        config.Timing0 = 0x00; /*Baud rate 500 Kbps  0x03  0x1C*/
        config.Timing1 = 0x1C;
        config.Mode = 0; //standard mode

        if (VCI_InitCAN(VCI_USBCAN2, 0, 0, & config) != 1) {
            throw_error("Init CAN1 error");
        }

        if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1) {
            throw_error("Start CAN1 error");
        }

        // Initialization motor
        for (int i = 1; i <= 7; i++) {
            send_motor(i, 2, 0, 0, 0, 0, 0, 0, 0, 0); //power on
            usleep(100000); // 50ms
        }
        send_motor(1, 2, 0, 0, 0, 0, 0, 0, 0, 0); //power on

        for (int i = 1540; i <= 1542; i++) //ID from 601 to 606
        {
            send_motor(i, 8, 43, 64, 96, 0, 1, 0, 0, 0); //reset
            usleep(100000); // 50ms
            //send_motor(1537, 8, 43, 64, 96, 0, 3, 0, 0, 0);//unlock break
            //sleep(0.5);
            send_motor(i, 8, 43, 64, 96, 0, 15, 0, 0, 0); //enable motor
            usleep(100000); // 50ms
        }

        ros::Rate loop_rate(40);
        while (ros::ok() && !sig_int_received) {
            ros::spinOnce();
            //read_cur_pos(10);
            read_cur_pos();
            read_cur_status();
            loop_rate.sleep();
        }
    } catch(const std::exception& e) {
        stringstream ss;
        ss << "An error has occurred: " << e.what();
        ROS_ERROR_STREAM(ss.str());
        return_status = -1;
    }

    ROS_INFO_STREAM("Closing CAN1 device");
    VCI_CloseDevice(VCI_USBCAN2, 0);

    return return_status;
}