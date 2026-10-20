#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <string>
#include <linux/can.h>

class CanInterface {
    public: 
        explicit CanInterface(const std::string& interfaceName); //Constructor
        ~CanInterface(); //Destructor

        //Forbid to have 2 obj with the same name and sockedFd
        CanInterface(const CanInterface&) = delete;
        CanInterface& operator=(const CanInterface&) = delete;

        void sendFrame(const can_frame& frame);

        void receiveFrame(can_frame& frame);

    private:
        int socketFd_{-1}; //Sets the value -1 if untouched inside the construtor before its first read
};

#endif