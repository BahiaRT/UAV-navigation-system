
/** @file socket.hpp
 *  @brief Socket Network Manager.
 *
 *  Contains functions to establish connection from the agents to the server and manage message exchanges.
 *
 *  @date 10/02/2023
 *  @author Alan Nascimento (bahiart)
 *  @author Glenda Santana (bahiart)
 *  @author Kalvin Albuquerque (bahiart)
 *
 */

#ifndef SOCKET_HPP
#define SOCKET_HPP

#include <string.h>
#include <iostream>
#include <vector>
#include <memory>
#include <cstring>

/*UNIX dependencies*/
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/poll.h>
#include <arpa/inet.h>

#define BUFFER_SIZE 2048

namespace bahiart::NetworkManager {
    /**
     * \brief General socket exception class, able to concatenate errors from UNIX native socket's functions (errno).
     *
     * The SocketException class will return a text message describing the specific exception that was thrown, providing an effective error handler.
     */
    class SocketException : std::exception {
    private:
        std::string *resultStr{};
        std::string description{};
        int errNum{};

        /**
         * \brief Concatenates response inside a newly created std::string, which gets pointed to by resultStr.
         *
         * \return Error message description. Returns error description + errno if the argument was passed, otherwise returns description alone.
         */
        void createResponse() {
            resultStr = new std::string(this->description + " -> " + gai_strerror(errNum));
        }

    public:
        /**
         * \brief Constructor of the SocketException Class.
         *
         * Receives error description.
         *
         * \param  description Description of error.
         */
        SocketException(std::string description) : description(description) {}

        /**
         * \brief Overload of constructor, called when errno argument is also setted.
         *
         * Receives error description and the just set global variable 'errno' by the failed function.
         *
         * \param description Description of error.
         * \param errNum The 'errno' set by the failed function
         * \sa description, errNum.
         */
        SocketException(std::string description, int errNum) : description(description), errNum(errNum) {
            this->createResponse();
        }

        /**
         * \brief Error message getter.
         *
         * Ensures errno was set by a failed function.
         *
         * \return Error message description. Returns error description + errno if the argument was passed, otherwise returns description alone.
         */
        const char *what() const throw() {
            // if (errNum != (int){} && errNum != 22) {
            //     return (*resultStr).c_str();
            // }
            // else {
            //     return this->description.c_str();
            // }
            return "";
        }

        ~SocketException() { delete (resultStr); }
    };

    /**
     * \brief Socket base abstract class.
     *
     * A pure virtual class to define the pattern functions and data members that will be inherited by TCP and UDP.
     *
     */
    class Socket {
    protected:
        /**
         * \brief Defines the socket type that will be used when setting the address and socket up.
         */
        int sockType{};

        /**
         * \brief Holds buffer message that is received by the server.
         */
        std::vector<char> buffer{};

        /**
         * \brief Socket File Descriptor, responsible for making system calls in order to provide network communication.
         *
         * \return A socket descriptor that can be used in later calls or -1 if an error occurs.
         */
        int socketFileDescriptor{};

        /**
         * \brief Points to a linked list of address informations struct.
         *
         * Holds all kinds of address information.
         */
        struct addrinfo *serverInfo{};

        /**
         * \brief Checks the socket's 'inbox'. Protected function.
         *
         * Is only called by receiveMessage() on the inner scope.
         *
         * \throw Exception: Error during checking messages from server.
         *
         * \return False if no ready data was found, true if there is data available to be read.
         */
        bool checkMessages();

    public:
        Socket();

        /**
         * \brief Sets up server address and socket's file descriptor.
         *
         * It receives the socket type from the superclass' sockType variable, defined in their children constructors
         *
         * \param HOST_NAME Host name or IP address to connect to.
         * \param PORT Port number to connect to.
         * \sa socketFileDescriptor
         * \throw SocketException: getaddrinfo() error.
         * \throw SocketException: Couldn't create file descriptor.
         */
        void setupAddress(const std::string HOST_NAME, const std::string PORT);

        void bindAddress(); 

        /**
         * \brief Attempts to establish connection to the server. Pure virtual function.
         */
        virtual void openConnection() = 0;

        /**
         * \brief Sends the string received as an argument to the server. Pure virtual function.
         *
         * It encodes the first four string bytes with the length/bytes of the following message (converted unsigned int).
         */
        virtual void sendMessage(std::string &message) = 0;

        /**
         * \brief Receives the messagem from the server. Pure virtual function.
         *
         * Reads the first four string bytes expecting the encoded length/bytes of the following message (converted unsigned int).
         * TCP has treatment for partial receives. It only stops when all the bytes sent from the server are stored.
         * The message is stored inside the class' char vector buffer, hopefully including the null terminator.
         *
         * \return Returns a "getter" for checkMessages(), true if there's a message, false if not.
         */
        virtual bool receiveMessage() = 0;

        /**
         * \brief Getter method for the buffer.
         *
         * \return String from the char vector buffer.
         */
        std::string getMessage();

    };

}

#endif