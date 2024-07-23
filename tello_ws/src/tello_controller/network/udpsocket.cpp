#include "udpsocket.hpp"
#include <fstream>

bahiart::NetworkManager::UdpSocket::UdpSocket() {
    this->sockType = SOCK_DGRAM;
}



void bahiart::NetworkManager::UdpSocket::sendMessage(std::string &message) {
    try {
        /* Initialize the string that will be sent to server */
        std::string str = message;

        std::cout << "bahiart -> sending message -> " << str << std::endl;

        /* Tries to send the message buffer from the just established connection, otherwise, throws an error. */
        if (sendto(socketFileDescriptor, str.data(), str.size(), 0, this->serverInfo->ai_addr, this->serverInfo->ai_addrlen) < 0)
        {
            throw SocketException("Couldn't send the message to the server", errno);
        }
    } catch (const bahiart::NetworkManager::SocketException &exception) {
        std::cerr << "SocketException - Error during socket messaging ---> " << exception.what() << std::endl;
        return;
    } catch (const std::exception &exception) {
        std::cerr << "std::exception - Default exception at sendMessage() ---> " << exception.what() << std::endl;
        return;
    }
}

bool bahiart::NetworkManager::UdpSocket::receiveMessage() {
    if (!checkMessages()) {
        return false;
    }

    /* Struct designed to be large enough to fit both ipv4 and ipv6 structures */
    struct sockaddr_storage fromaAddr{};

    /* Type designed to hold length of socket structures. */
    socklen_t fromLen = sizeof fromaAddr;


    try {

        /* Cleaning buffer before using */
        this->buffer.clear();

        /* Writing the message in buffer vector */
        if (recvfrom(this->socketFileDescriptor, this->buffer.data(), this->buffer.capacity(), 0, (struct sockaddr *)&fromaAddr, &fromLen) < 0) {
            throw bahiart::NetworkManager::SocketException("recvfrom()");
        }
        
        return true;
    } catch (bahiart::NetworkManager::SocketException &exception) {
        std::cerr << "SocketException - Error during receiving messages ---> " << exception.what() << std::endl;
        return false;
    } catch (std::exception &exception) {
        std::cerr << "std::exception - Default exception at receiveMessage()" << std::endl;
        return false;
    }
}

bahiart::NetworkManager::UdpSocket::~UdpSocket() {
    freeaddrinfo(this->serverInfo); // getaddrinfo()'s linked tree, used in struct addrinfo *serverInfo, is freed;
}


