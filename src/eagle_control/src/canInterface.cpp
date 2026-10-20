#include "eagle_control/canInterface.hpp"
#include <string>

#include <cerrno>
#include <stdexcept>
#include <system_error>

#include <sys/socket.h> //Linux socket api
#include <linux/can.h> //CAN consts
#include <linux/can/raw.h> //RAW
#include <unistd.h> //close()

#include <net/if.h> //network lookup

//NOTE - CanInterface Contructor
CanInterface::CanInterface(const std::string& interfaceName) {
    //Socket creation socket(Protocol Family, CAN Frames, CAN Protocol)
    socketFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (socketFd_ < 0) { //socket() returns -1 on failure
        throw std::runtime_error("CAN Socket creation failed");
    }

    const unsigned int interfaceIdx = if_nametoindex(interfaceName.c_str()); //Converting strig to cstring ( \0 alla fine )

    if (!interfaceIdx) {
        close(socketFd_); //STACCa tutto se qualcosa non è andato
        throw std::runtime_error("CAN Interface Idx not found");
    }

    struct sockaddr_can address{};
    address.can_family = AF_CAN; //CAN endpoint
    address.can_ifindex = interfaceIdx;

    //Associa il socket all'interfaccia di "vcan0", poi converte da pointer "sockaddr_can*"
    //quindi il tipo che ha ora address, a "sockaddr*"
    if (bind(socketFd_, reinterpret_cast<const struct sockaddr*>(&address), sizeof(address)) < 0) {
        close(socketFd_);
        throw std::runtime_error("Bind del CANe non riuscito");
    }
}

//NOTE - CanInterface::sendFrame Function
void CanInterface::sendFrame(const can_frame& frame) {
    ssize_t writtenBytes = write(socketFd_, &frame, sizeof(frame));

    if (writtenBytes == -1) { 
        throw std::system_error(
            errno,
            std::generic_category(),
            "CAN write failed"
        );
    }

    if (writtenBytes != sizeof(frame)) { 
        throw std::runtime_error("Frame size doesn't match");
    }
}

//NOTE - CanInterface::receiveFrame Function
void CanInterface::receiveFrame(can_frame& frame) {
    ssize_t receivedBytes = read(socketFd_, &frame, sizeof(frame));

    if (receivedBytes == -1) { 
        throw std::system_error(
            errno,
            std::generic_category(),
            "CAN read failed"
        );
    }

    if (receivedBytes != sizeof(frame)) { 
        throw std::runtime_error("Frame size doesn't match");
    }
}

//NOTE - CanInterface Destructor
CanInterface::~CanInterface() {
    if (socketFd_ >= 0) {
        close(socketFd_);
    }
}