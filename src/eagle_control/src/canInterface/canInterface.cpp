#include "canInterface.hpp"
#include <string>

#include <cerrno>
#include <stdexcept>
#include <system_error>

#include <sys/socket.h> //Linux socket api
#include <linux/can.h> //CAN consts
#include <linux/can/raw.h> //RAW
#include <unistd.h> //close()

#include <net/if.h> //network lookup

CanInterface::CanInterface(const std::string& interfaceName) {
    //Socket creation -- Protocol Family -- CAN Frames -- Raw CAN Protocol
    socketFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (socketFd_ < 0) { //socket() returns -1 on failure
        throw std::runtime_error("CAN Socket creation failed");
    }

    const unsigned int interfaceIdx = if_nametoindex(interfaceName.c_str()); //Converting strig to cstring ( \0 alla fine )

    if (!interfaceIdx) {
        close(socketFd_); //STACCa tutto se qualcosa non è andato
        throw std::runtime_error("CAN Interface Idx not found");
    }

    struct sockaddr_can address{}; //Inizializza a zero tutto lo struct
    address.can_family = AF_CAN; //CAN endpoint
    address.can_ifindex = interfaceIdx;

    //Associa il socket all'interfaccia di "vcan0", pos converte da pointer "sockaddr_can*" a "sockaddr*"
    if (bind(socketFd_, reinterpret_cast<const struct sockaddr*>(&address), sizeof(address)) < 0) {
        close(socketFd_);
        throw std::runtime_error("Bind del CANe non riuscito");
    }
}

void CanInterface::sendFrame(const can_frame& frame) {
    ssize_t writtenBytes = write(socketFd_, &frame, sizeof(frame));

    if (writtenBytes == -1) { 
        throw std::system_error(
            errno,
            std::generic_category(),
            "CAN write failed"
        );
    }

    //Controllo se quello che mando è = a quello che effettivamente voglio mandare
    if (writtenBytes != sizeof(frame)) { 
        throw std::runtime_error("Frame size doesn't match");
    }
}

void CanInterface::receiveFrame(can_frame& frame) {
    ssize_t receivedBytes = read(socketFd_, &frame, sizeof(frame));

    if (receivedBytes == -1) { 
        throw std::system_error(
            errno,
            std::generic_category(),
            "CAN read failed"
        );
    }

    //Controllo se quello che mando è = a quello che effettivamente voglio mandare
    if (receivedBytes != sizeof(frame)) { 
        throw std::runtime_error("Frame size doesn't match");
    }
}

CanInterface::~CanInterface() {
    if (socketFd_ >= 0) {
        close(socketFd_);
    }
}