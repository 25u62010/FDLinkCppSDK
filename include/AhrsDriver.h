#include <iostream>
#include <string>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "crc_table.h"
#include "fdilink_data_struct.h"
#include <ostream>
using namespace std;
namespace FDILink{ 

#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN  0x38   //56
#define AHRS_LEN 0x30   //48
#define INSGPS_LEN 0x54 //84
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295

struct AhrsDataStruct{
    int64_t timeStamp;
    double qw;
    double qx;
    double qy;
    double qz;
    double rollSpeed;
    double pitchSpeed;
    double yawSpeed;
    double linearAccelerometerX;
    double linearAccelerometerY;
    double linearAccelerometerZ;
    void operator=(const AhrsDataStruct& other){
        timeStamp = other.timeStamp;
        qw = other.qw;
        qx = other.qx;
        qy = other.qy;
        qz = other.qz;
        rollSpeed = other.rollSpeed;
        pitchSpeed = other.pitchSpeed;
        yawSpeed = other.yawSpeed;
        linearAccelerometerX = other.linearAccelerometerX;
        linearAccelerometerY = other.linearAccelerometerY;
        linearAccelerometerZ = other.linearAccelerometerZ;
    }
};

class AhrsDriver
{
private:
    serial::Serial mSerial;
    thread* mpReadThread=nullptr;
    mutex mStopMutex;
    mutex mFinishMutex;
    mutex mDataMutex;
    uint8_t mReadSn=0;
    int mSnLost=0;
    bool mbFirstSn=false;
    bool mbStoped=false;
    bool mbFinished=false;
    void run();
    bool checkFinish();
    void setFinished();
    AhrsDataStruct mLastData;
    list<AhrsDataStruct> mDataBuffer;
    FDILink::imu_frame_read  mImuFrame;
    FDILink::ahrs_frame_read mAhrsFrame;
    FDILink::insgps_frame_read mInsgpsFrame;
    void checkSN(int type);
    void writeBuffer(AhrsDataStruct& data);
public:
    AhrsDriver();
    bool OpenSerial(string configFilesPath);
    void StartRead();
    void FinishRead();
    bool isFinished();
    void readBuffer(AhrsDataStruct& data);
    friend ostream& operator<<(ostream &out,AhrsDriver& obj){
        AhrsDataStruct dataNew;
        obj.readBuffer(dataNew);
        out<<
        "Time stamp:"
        <<dataNew.timeStamp<<endl<<
        "Linear accleromter:"<<
        dataNew.linearAccelerometerX<<" "<<dataNew.linearAccelerometerY<<" "<<dataNew.linearAccelerometerZ<<endl<<
        "Quaternion:"<<
        dataNew.qw<<" "<<dataNew.qx<<" "<<dataNew.qy<<dataNew.qz<< endl<<
        "Angular speed:"<<
        dataNew.rollSpeed<<" "<<dataNew.pitchSpeed<<" "<<dataNew.yawSpeed<<endl<<
        "------------------------------------------------------------------";
        return out;
    }
    friend ofstream& operator<<(ofstream &outFile,AhrsDriver& obj){
        AhrsDataStruct dataNew;
        obj.readBuffer(dataNew);
        outFile<<
        dataNew.timeStamp<<" "<<
        dataNew.linearAccelerometerX<<" "<<dataNew.linearAccelerometerY<<" "<<dataNew.linearAccelerometerZ<<" "<<
        dataNew.qw<<" "<<dataNew.qx<<" "<<dataNew.qy<<dataNew.qz<< " "<<
        dataNew.rollSpeed<<" "<<dataNew.pitchSpeed<<" "<<dataNew.yawSpeed;
        return outFile;
    }
    ~AhrsDriver();
};
}