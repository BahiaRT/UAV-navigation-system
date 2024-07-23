#include "tcpsocket.hpp"

bahiart::NetworkManager::TcpClient::TcpClient()
{
    this->sockType = SOCK_STREAM;
}

bahiart::NetworkManager::TcpClient::~TcpClient()
{
    close(this->socketFileDescriptor); // closes connection to the server
    freeaddrinfo(this->serverInfo);    // getaddrinfo()'s linked tree, used in struct addrinfo *serverInfo, is freed;
}

void bahiart::NetworkManager::TcpClient::openConnection()
{
    try
    {
        if (connect(this->socketFileDescriptor, this->serverInfo->ai_addr, this->serverInfo->ai_addrlen) < 0)
        {
            throw bahiart::NetworkManager::SocketException("Couldn't connect to remote host", errno);
        }
        return;
    }

    catch (bahiart::NetworkManager::SocketException& exception)
    {
        std::cerr << "SocketException - Error during socket connection ---> " << exception.what() << std::endl;
        return;
    }

    catch (std::exception& exception)
    {
        std::cerr << "std::exception - Default exception at openConnection() --->" << exception.what() << std::endl;
        return;
    }

    return;
}

void bahiart::NetworkManager::TcpClient::sendMessage(std::string &message)
{
    try
    {      

        /* Tries to send the message buffer from the just established connection, otherwise, throws an error. */
        if (send(this->socketFileDescriptor, message.data(), message.size(), 0) < 0){
            throw SocketException("Couldn't send the message to the server", errno);
        }
        
    }

    catch (bahiart::NetworkManager::SocketException& exception)
    {
        std::cerr << "SocketException - Error during socket messaging ---> " << exception.what() << std::endl;
        return;
    }

    catch (std::exception& exception)
    {
        std::cerr << "std::exception - Default exception at sendMessage()" << exception.what() << std::endl;
        return;
    }

}

bool bahiart::NetworkManager::TcpClient::receiveMessage()
{
    if(!checkMessages()){
        return false;
    }

    try {
        
        /* Cleaning buffer before using */
        this->buffer.clear();
        
        recv(this->socketFileDescriptor, this->buffer.data(), BUFFER_SIZE, 0);
        
        return true;
    }
    catch (bahiart::NetworkManager::SocketException& exception)
    {
        std::cerr << "SocketException - Error during receiving messages ---> " << exception.what() << std::endl;
        return false;
    }

    catch (std::exception& exception)
    {
        std::cerr << "std::exception - Default exception at receiveMessage()" << std::endl;
        return false;
    }
    
}

bahiart::NetworkManager::TcpServer::TcpServer()
{
    this->sockType = SOCK_STREAM;
}

void bahiart::NetworkManager::TcpServer::sendMessage(std::string &message)
{
    try
    {      

        /* Tries to send the message buffer from the just established connection, otherwise, throws an error. */
        if (send(this->theirFileDescriptor, message.data(), message.size(), 0) < 0){
            throw SocketException("Couldn't send the message to the server", errno);
        }
        
    }

    catch (bahiart::NetworkManager::SocketException& exception)
    {
        std::cerr << "SocketException - Error during socket messaging ---> " << exception.what() << std::endl;
        return;
    }

    catch (std::exception& exception)
    {
        std::cerr << "std::exception - Default exception at sendMessage()" << exception.what() << std::endl;
        return;
    }
}

bahiart::NetworkManager::TcpServer::~TcpServer()
{
    close(this->socketFileDescriptor); // closes connection to the server
    freeaddrinfo(this->serverInfo);    // getaddrinfo()'s linked tree, used in struct addrinfo *serverInfo, is freed;
}


void bahiart::NetworkManager::TcpServer::openConnection()
{

    try
    {
        if (listen(this->socketFileDescriptor, BACKLOG) < 0)
        {
            throw bahiart::NetworkManager::SocketException("Couldn't connect to remote host", errno);
        }
        return;
    }

    catch (bahiart::NetworkManager::SocketException& exception)
    {
        std::cerr << "SocketException - Error during socket connection ---> " << exception.what() << std::endl;
        return;
    }

    catch (std::exception& exception)
    {
        std::cerr << "std::exception - Default exception at openConnection() --->" << exception.what() << std::endl;
        return;
    }

    return;
}

void bahiart::NetworkManager::TcpServer::acceptConnection()
{

    /* Struct designed to be large enough to fit both ipv4 and ipv6 structures */
    struct sockaddr_storage fromaAddr{};

    /* Type designed to hold length of socket structures. */
    socklen_t fromLen = sizeof fromaAddr;

    try {
     
        this->theirFileDescriptor = accept(this->socketFileDescriptor, (struct sockaddr *)&fromaAddr, &fromLen);
        
        return;
    }
    catch (bahiart::NetworkManager::SocketException& exception)
    {
        std::cerr << "SocketException - Error during receiving messages ---> " << exception.what() << std::endl;
        return ;
    }

    catch (std::exception& exception)
    {
        std::cerr << "std::exception - Default exception at receiveMessage()" << std::endl;
        return ;
    }

}

bool bahiart::NetworkManager::TcpServer::receiveMessage()
{
    if(!checkMessages()){
        return false;
    }
    std::cout << "lendo mensagem" << std::endl; 
    try {
        
        /* Cleaning buffer before using */
        this->buffer.clear();
        
        recv(this->theirFileDescriptor, this->buffer.data(), BUFFER_SIZE, 0);
        
        return true;
    }
    catch (bahiart::NetworkManager::SocketException& exception)
    {
        std::cerr << "SocketException - Error during receiving messages ---> " << exception.what() << std::endl;
        return false;
    }

    catch (std::exception& exception)
    {
        std::cerr << "std::exception - Default exception at receiveMessage()" << std::endl;
        return false;
    }
}
