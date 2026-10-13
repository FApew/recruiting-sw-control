#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <string>
#include <linux/can.h>

class CanInterface {
    public: 
        explicit CanInterface(const std::string& interfaceName); //Constructor
        ~CanInterface(); //Destructor

        //Impedisce la copia
        CanInterface(const CanInterface&) = delete;
        CanInterface& operator=(const CanInterface&) = delete;

        void sendFrame(const can_frame& frame);

        void receiveFrame(can_frame& frame);

    private:
        int socketFd_{-1}; //If the constructor non modifica il valore, allora lui lo setta a -1
};

#endif