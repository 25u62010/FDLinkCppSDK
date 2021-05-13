#include <iostream>
#include <string>
#include "AhrsDriver.h"
#include <unistd.h>
#include <ostream>
using namespace std;
using namespace FDILink;
int main(){
    AhrsDriver test;
    test.OpenSerial("../param/config.yaml");
    test.StartRead();
    AhrsDataStruct newData;
    ofstream outputFile;
    outputFile.open("outData.txt");
    while(true){
        cout<<test<<endl;
        //outputFile<<test<<endl;
        usleep(500);
    }
    test.FinishRead();
    while(!test.isFinished()){
        usleep(500);
    }
    cout<<"exit success"<<endl;
}