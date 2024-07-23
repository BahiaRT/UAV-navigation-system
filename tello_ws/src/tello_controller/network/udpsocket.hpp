/** @file udpsocket.hpp
 *  @brief Network Manager class inherited from Socket to connect through UDP protocol.
 *
 *  Contains functions to establish connection from the agents to the server and manage message exchanges.
 *  
 *  @date 10/02/2023
 *  @author Alan Nascimento (bahiart)
 *  @author Glenda Santana (bahiart)
 *  @author Kalvin Albuquerque (bahiart)
 *  
 */

#ifndef UDP_SOCKET_HPP
#define UDP_SOCKET_HPP

#include "socket.hpp"

namespace bahiart::NetworkManager {   
    /**
     * \brief Network Manager class inherited from Socket to connect through UDP protocol.
     *
     * The UdpSocket class has functions to establish connection from the agents to the server and manage message exchanges.
     * 
     */
    class UdpSocket : public bahiart::NetworkManager::Socket {
    public:
        /**
         * \brief Sets superclass' socket type used in address setup
         */
        UdpSocket();
        
        /**
         * \brief Sends message to the stored socket descriptor.
         * 
         * \param message String message that will be sent to server.
         * 
         * \throw Exception: Couldn't send the message to the server.
         */
        void sendMessage(std::string& message) override;

        /**
         * \brief Reads messages from the server. 
         *
         * \return True if there is message ready to be read, false if there isn't any message.
         * 
         * \throw Exception: Undefined message length.
         */
        bool receiveMessage() override;

        /**
         * \brief Closes connection to remote host (socket descriptor).
         *
         * Frees stored addrinfo's linked tree.
         */
        ~UdpSocket();
        
    private:
        /**
         * \brief Forced implementation due to pure virtual base class -> connection is not needed in UDP
         */
        void openConnection() override {};
    };

};

#endif