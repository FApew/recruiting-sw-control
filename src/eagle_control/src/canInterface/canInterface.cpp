#include "canInterface.hpp"
#include <string>
#include <stdexcept>

#include <sys/socket.h> //Linux socket api
#include <linux/can.h> //CAN consts
#include <linux/can/raw.h> //RAW

#include <net/if.h> //network lookup



CanInterface::CanInterface(const std::string& interfaceName) {
    //Socket creation -- Protocol Family -- CAN Frames -- Raw CAN Protocol
    socketFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (socketFd_ < 0) { //socket() returns -1 on failure
        throw std::runtime_error("CAN Socket creation failed");
    }

    const unsigned int interfaceIdx = if_nametoindex(interfaceName.c_str()); //Converting strig to cstring ( \0 alla fine )

    if (!interfaceIdx) {
        throw std::runtime_error("CAN Interface Idx not found");
    }
}

int main() {
    return 0;
}