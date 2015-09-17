
#include "Serial.h"





Serial::Serial()
{


}

  // destructor
Serial::~Serial() 
{

}

// send stuff
int Serial::writeBuffer(void *buffer, int len)
{
    return 0;//write (serial_fd, buffer, len);*/
}

  // receive stuff
int Serial::readBuffer(void *buffer, int bufferSize)
{
    return 0;//read(serial_fd, buffer, bufferSize);//sizeof(buffer));*/
}


