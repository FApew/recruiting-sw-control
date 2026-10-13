#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <string>

class CanInterface {
    public: 
        explicit CanInterface(const std::string& interfaceName); //Constructor
        ~CanInterface(); //Destructor

    private:
        int socketFd_;
};

#endif