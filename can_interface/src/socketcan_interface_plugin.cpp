#include <class_loader/class_loader.h> 
#include <can_interface/socketcan_driver.h>

CLASS_LOADER_REGISTER_CLASS(can::SocketCANInterface, can::SocketCANDriverInterface);
