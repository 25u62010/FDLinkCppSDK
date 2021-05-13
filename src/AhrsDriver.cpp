#include "AhrsDriver.h"
using namespace std;
using namespace FDILink;
AhrsDriver::AhrsDriver()
{
}
bool AhrsDriver::OpenSerial(string configFilePath){
    assert(!configFilePath.empty());
    try{
        cv::FileStorage configFile(configFilePath,cv::FileStorage::READ);
        string serialPortName=configFile["serial.port"];
        int serialBaudrate=configFile["serial.baudrate"];
        mSerial.setPort(serialPortName);
        mSerial.setBaudrate(serialBaudrate);
        mSerial.setFlowcontrol(serial::flowcontrol_none);
        mSerial.setParity(serial::parity_none);
        mSerial.setStopbits(serial::stopbits_one);
        mSerial.setBytesize(serial::eightbits);
        serial::Timeout timeOut=serial::Timeout::simpleTimeout(20);
        mSerial.setTimeout(timeOut);
        mSerial.open();
    }
    catch(cv::Exception& exc){
        cout<<"Config file error."<<endl;
        return false;
    }
    catch(serial::IOException& exc){
        cout<<"Unable to open port."<<endl;
        return false;
    }
    if(mSerial.isOpen()){
        cout<<"Serial port initialized."<<endl;
        return false;
    }
    else{
        cout<<"Unable to open port."<<endl;
    }
    return true;
}
void AhrsDriver::StartRead(){
    assert(mpReadThread==nullptr);
    mpReadThread=new thread(&AhrsDriver::run,this);
}   
bool AhrsDriver::checkFinish(){
    unique_lock<mutex> lock(mStopMutex);
    return mbStoped;
}
void AhrsDriver::FinishRead(){
    unique_lock<mutex> lock(mStopMutex);
    mbStoped=true;
}
bool AhrsDriver::isFinished(){
    unique_lock<mutex> lock(mFinishMutex);
    if(mbFinished){
        delete mpReadThread;
    }
    return mbFinished;   
}
void  AhrsDriver::setFinished(){
    unique_lock<mutex> lock(mFinishMutex);
    mbFinished=true;       
}
void AhrsDriver::run(){
    assert(mSerial.isOpen());
    while(true){
        if(checkFinish()){
            mSerial.close();
            break;
        }
        uint8_t checkHead[1] = {0xff};
        size_t head_s = mSerial.read(checkHead, 1);     
        if (checkHead[0] != FRAME_HEAD){
            this_thread::sleep_for(chrono::microseconds(1));
            continue;
        } 
        uint8_t headType[1] = {0xff};
        size_t typeS = mSerial.read(headType, 1);
        if (headType[0] != TYPE_IMU && headType[0] != TYPE_AHRS 
            && headType[0] != TYPE_INSGPS && headType[0] != 0x50 
            && headType[0] != TYPE_GROUND){
            cout<<"headType error:"<<headType[0]<<endl;
            continue;
        }
        uint8_t checkLen[1] = {0xff};
        size_t len_s = mSerial.read(checkLen, 1);
        if (headType[0] == TYPE_IMU && checkLen[0] != IMU_LEN){
            cout<<"head_len error (imu)"<<endl;
            continue;
        }
        else if (headType[0] == TYPE_AHRS && checkLen[0] != AHRS_LEN){
            cout<<"head_len error (ahrs)"<<endl;
            continue;
        }
        else if (headType[0] == TYPE_INSGPS && checkLen[0] != INSGPS_LEN){
            cout<<"head_len error (insgps)"<<endl;
            continue;
        }
        else if (headType[0] == TYPE_GROUND || headType[0] == 0x50){
            uint8_t groundSn[1];
            size_t groundSnS = mSerial.read(groundSn, 1);
            if (++mReadSn != groundSn[0]){
                if ( groundSn[0] < mReadSn){
                    mSnLost += 256 - (int)(mReadSn - groundSn[0]);
                    mReadSn = groundSn[0];
                    // continue;
                }
                else
                {
                    mSnLost += (int)(groundSn[0] - mReadSn);
                    mReadSn = groundSn[0];
                    // continue;
                }
            }
            uint8_t groundIgnore[500];
            size_t groundIgnoreS = mSerial.read(groundIgnore, (checkLen[0]+4));
            continue;
        }
        //read head sn 
        uint8_t checkSn[1] = {0xff};
        size_t sn_s = mSerial.read(checkSn, 1);
        uint8_t headCrc8[1] = {0xff};
        size_t crc8_s = mSerial.read(headCrc8, 1);
        uint8_t headCrc16H[1] = {0xff};
        uint8_t headCrc16L[1] = {0xff};
        size_t crc16HS = mSerial.read(headCrc16H, 1);
        size_t crc16LS = mSerial.read(headCrc16L, 1);
        if (headType[0] == TYPE_IMU){
            mImuFrame.frame.header.header_start   = checkHead[0];
            mImuFrame.frame.header.data_type      = headType[0];
            mImuFrame.frame.header.data_size      = checkLen[0];
            mImuFrame.frame.header.serial_num     = checkSn[0];
            mImuFrame.frame.header.header_crc8    = headCrc8[0];
            mImuFrame.frame.header.header_crc16_h = headCrc16H[0];
            mImuFrame.frame.header.header_crc16_l = headCrc16L[0];
            uint8_t CRC8 = CRC8_Table(mImuFrame.read_buf.frame_header, 4);
            if (CRC8 != mImuFrame.frame.header.header_crc8)
            {
                cout<<"header_crc8 error"<<endl;
                continue;
            }
            if(!mbFirstSn){
                mReadSn  = mImuFrame.frame.header.serial_num - 1;
                mbFirstSn = true;
            }
            //check sn 
            checkSN(TYPE_IMU);
        }
        else if (headType[0] == TYPE_AHRS){
            mAhrsFrame.frame.header.header_start   = checkHead[0];
            mAhrsFrame.frame.header.data_type      = headType[0];
            mAhrsFrame.frame.header.data_size      = checkLen[0];
            mAhrsFrame.frame.header.serial_num     = checkSn[0];
            mAhrsFrame.frame.header.header_crc8    = headCrc8[0];
            mAhrsFrame.frame.header.header_crc16_h = headCrc16H[0];
            mAhrsFrame.frame.header.header_crc16_l = headCrc16L[0];
            uint8_t CRC8 = CRC8_Table(mAhrsFrame.read_buf.frame_header, 4);
            if (CRC8 != mAhrsFrame.frame.header.header_crc8){
                cout<<"header_crc8 error"<<endl;
                continue;
            }
            if(!mbFirstSn){
                mReadSn  = mAhrsFrame.frame.header.serial_num - 1;
                mbFirstSn = true;
            }
            //check sn 
            checkSN(TYPE_AHRS);
        }
        else if (headType[0] == TYPE_INSGPS){
            mInsgpsFrame.frame.header.header_start   = checkHead[0];
            mInsgpsFrame.frame.header.data_type      = headType[0];
            mInsgpsFrame.frame.header.data_size      = checkLen[0];
            mInsgpsFrame.frame.header.serial_num     = checkSn[0];
            mInsgpsFrame.frame.header.header_crc8    = headCrc8[0];
            mInsgpsFrame.frame.header.header_crc16_h = headCrc16H[0];
            mInsgpsFrame.frame.header.header_crc16_l = headCrc16L[0];
            uint8_t CRC8 = CRC8_Table(mInsgpsFrame.read_buf.frame_header, 4);
            if (CRC8 != mInsgpsFrame.frame.header.header_crc8){
                cout<<"header_crc8 error"<<endl;
                continue;
            }      
            checkSN(TYPE_INSGPS);
        }

        if (headType[0] == TYPE_IMU){
            uint16_t headCrc16L = mImuFrame.frame.header.header_crc16_l;
            uint16_t headCrc16H = mImuFrame.frame.header.header_crc16_h;
            uint16_t head_crc16 = headCrc16L + (headCrc16H << 8);
            size_t data_s = mSerial.read(mImuFrame.read_buf.read_msg, (IMU_LEN + 1)); //48+1
            uint16_t CRC16 = CRC16_Table(mImuFrame.frame.data.data_buff, IMU_LEN);
            
            if (head_crc16 != CRC16){
                cout<<"check crc16 faild(imu)."<<endl;
                continue;
            }
            else if(mImuFrame.frame.frame_end != FRAME_END){
                cout<<"check frame end."<<endl;
                continue;
            }
        }
        else if (headType[0] == TYPE_AHRS)
        {
            uint16_t headCrc16L = mAhrsFrame.frame.header.header_crc16_l;
            uint16_t headCrc16H = mAhrsFrame.frame.header.header_crc16_h;
            uint16_t head_crc16 = headCrc16L + (headCrc16H << 8);
            size_t data_s = mSerial.read(mAhrsFrame.read_buf.read_msg, (AHRS_LEN + 1)); //48+1
            uint16_t CRC16 = CRC16_Table(mAhrsFrame.frame.data.data_buff, AHRS_LEN);
            
            if (head_crc16 != CRC16){
                cout<<"check crc16 faild(ahrs)."<<endl;
                continue;
            }
            else if(mAhrsFrame.frame.frame_end != FRAME_END){
                cout<<"check frame end."<<endl;
                continue;
            }
        }
        else if (headType[0] == TYPE_INSGPS){
            uint16_t head_crc16 = mInsgpsFrame.frame.header.header_crc16_l + ((uint16_t)mInsgpsFrame.frame.header.header_crc16_h << 8);
            size_t data_s = mSerial.read(mInsgpsFrame.read_buf.read_msg, (INSGPS_LEN + 1)); //48+1
            uint16_t CRC16 = CRC16_Table(mInsgpsFrame.frame.data.data_buff, INSGPS_LEN);
            if (head_crc16 != CRC16){
                cout<<"check crc16 faild(insgps)."<<endl;
                continue;
            }
            else if(mInsgpsFrame.frame.frame_end != FRAME_END){
                cout<<"check frame end."<<endl;
                continue;
            }
        }
        if(headType[0]==TYPE_AHRS){
            AhrsDataStruct newdata;
            newdata.timeStamp=mImuFrame.frame.data.data_pack.Timestamp;
            newdata.qw=mAhrsFrame.frame.data.data_pack.Qw;
            newdata.qx=mAhrsFrame.frame.data.data_pack.Qx;
            newdata.qy=mAhrsFrame.frame.data.data_pack.Qy;
            newdata.qz=mAhrsFrame.frame.data.data_pack.Qz;
            newdata.rollSpeed=mAhrsFrame.frame.data.data_pack.RollSpeed;
            newdata.pitchSpeed=mAhrsFrame.frame.data.data_pack.PitchSpeed;
            newdata.yawSpeed=mAhrsFrame.frame.data.data_pack.HeadingSpeed;
            newdata.linearAccelerometerX=mImuFrame.frame.data.data_pack.accelerometer_x;
            newdata.linearAccelerometerY=mImuFrame.frame.data.data_pack.accelerometer_y;
            newdata.linearAccelerometerZ=mImuFrame.frame.data.data_pack.accelerometer_z;
            writeBuffer(newdata);
        }
    }//while(true)
    setFinished();
} 
void AhrsDriver::checkSN(int type)
{
  switch (type)
  {
  case TYPE_IMU:
    if (++mReadSn != mImuFrame.frame.header.serial_num){
      if ( mImuFrame.frame.header.serial_num < mReadSn){
        mSnLost += 256 - (int)(mReadSn - mImuFrame.frame.header.serial_num);
      }
      else
      {
        mSnLost += (int)(mImuFrame.frame.header.serial_num - mReadSn);
      }
    }
    mReadSn = mImuFrame.frame.header.serial_num;
    break;

  case TYPE_AHRS:
    if (++mReadSn != mAhrsFrame.frame.header.serial_num)
    {
      if ( mAhrsFrame.frame.header.serial_num < mReadSn)
      {
        mSnLost += 256 - (int)(mReadSn - mAhrsFrame.frame.header.serial_num);
      }
      else
      {
        mSnLost += (int)(mAhrsFrame.frame.header.serial_num - mReadSn);
      }
    }
    mReadSn = mAhrsFrame.frame.header.serial_num;
    break;

  case TYPE_INSGPS:
    if (++mReadSn != mInsgpsFrame.frame.header.serial_num)
    {
      if ( mInsgpsFrame.frame.header.serial_num < mReadSn)
      {
        mSnLost += 256 - (int)(mReadSn - mInsgpsFrame.frame.header.serial_num);
      }
      else
      {
        mSnLost += (int)(mInsgpsFrame.frame.header.serial_num - mReadSn);
      }
    }
    mReadSn = mInsgpsFrame.frame.header.serial_num;
    break;

  default:
    break;
  }
}
void AhrsDriver::writeBuffer(AhrsDataStruct& data){
    unique_lock<mutex> lock(mDataMutex);
    if(mDataBuffer.size()>100){
        mDataBuffer.pop_front();
    }
    mLastData = data;
    mDataBuffer.push_back(data);
}
void AhrsDriver::readBuffer(AhrsDataStruct& data){
    unique_lock<mutex> lock(mDataMutex);
    if(mDataBuffer.size()==0){
        data = mLastData;
    }
    else{
        data = mDataBuffer.front();
        mDataBuffer.pop_front();
    }
}

AhrsDriver::~AhrsDriver()
{
    
}
