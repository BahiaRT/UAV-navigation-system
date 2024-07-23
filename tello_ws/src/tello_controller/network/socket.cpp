#include "socket.hpp"

bahiart::NetworkManager::Socket::Socket()
{
    this->buffer.resize(BUFFER_SIZE);
}

bool bahiart::NetworkManager::Socket::checkMessages() {
    /*
    Object that holds data structure describing a polling request.
    */
    struct pollfd ufds;
    ufds.fd = this->socketFileDescriptor;
    ufds.events = POLLIN;
    /* Sets the type of event that poll() will be waiting to happen -> POLLIN stands for normal data */

    /*
    The rv variable will receive the output of poll() function, it could be:
        -1 for error
        0 for no event occurred in socketfiledescriptor during the limit of waiting time
        >0 for the number of events occurred in the socketfiledescriptor
    */
    int rv{};

    try {
        /*
        poll() receives 3 parameters:
            the address of the object that keep the struct of pollfd
            the number of objects that poll() will be following
            limit of waiting time that the function will wait for events (-1 makes it wait forever)
        */
        rv = poll(&ufds, 1, 0);

        if (rv < 0) {
            throw bahiart::NetworkManager::SocketException("Error during checking messages from server --> poll()", errno);
        }

        /* Retuns true if poll detects incoming data */
        return (rv > 0 && (ufds.revents & POLLIN));
    } catch (bahiart::NetworkManager::SocketException &exception) {
        std::cerr << "SocketException - Error during checking messages ---> " << exception.what() << std::endl;
        return false;
    } catch (std::exception &exception) {
        std::cerr << "std::exception - Default exception at checkMessage()" << exception.what() << std::endl;
        return false;
    }
}

void bahiart::NetworkManager::Socket::setupAddress(const std::string HOST_NAME, const std::string PORT) {
    int addrStatus{};
    struct addrinfo hints{};

    try {
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = this->sockType;

        /* Getting server info from the initial addrinfo (hints), HOST/IP and port - getaddrinfo() */
        if ((addrStatus = getaddrinfo(HOST_NAME.c_str(), PORT.c_str(), &hints, &(this->serverInfo))) != 0) {
            // tests may be needed for port length ---> gai_strerror() exceptions not clear
            throw bahiart::NetworkManager::SocketException(gai_strerror(addrStatus));
        }

        /* File Descriptor/Socket Descriptor created based on the serverInfo - socket() */
        if ((this->socketFileDescriptor = socket(serverInfo->ai_family, serverInfo->ai_socktype, serverInfo->ai_protocol)) < 0) {
            throw bahiart::NetworkManager::SocketException("Couldn't create file descriptor", errno);
        }

        return;
    } catch (bahiart::NetworkManager::SocketException &exception) {
        std::cerr << "SocketException - Error during socket creation or address setup ---> " << exception.what() << std::endl;
        return;
    } catch (std::exception &exception) {
        std::cerr << "std::exception - Default exception at setupAddress()" << exception.what() << std::endl;
        return;
    }
}

void bahiart::NetworkManager::Socket::bindAddress() {

    try{
        if ((bind(this->socketFileDescriptor, this->serverInfo->ai_addr, this->serverInfo->ai_addrlen)) < 0) {
            throw SocketException("Couldn't bind the UDP socket", errno);
        } 
    }
    catch (const bahiart::NetworkManager::SocketException &exception){
        std::cerr << "SocketException - Error during socket binding ---> " << exception.what() << std::endl;
        exit(0);
    }
    
}


std::string bahiart::NetworkManager::Socket::getMessage() {
    return (std::string)this->buffer.data();
}
