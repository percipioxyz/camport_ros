#ifndef _PERCIPIO_LOG_SERVER_H_
#define _PERCIPIO_LOG_SERVER_H_

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <functional>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

class PercipioTcpLogServer {
public:
    // Log callback type
    using LogCallback = std::function<void(const std::string& message)>;

    // Get singleton instance
    static PercipioTcpLogServer& getInstance() {
        static PercipioTcpLogServer instance;
        return instance;
    }

    // Delete copy constructor and assignment operator
    PercipioTcpLogServer(const PercipioTcpLogServer&) = delete;
    PercipioTcpLogServer& operator=(const PercipioTcpLogServer&) = delete;

    // Start the server
    bool start(int port = 9000);

    // Stop the server
    void stop();

    // Check if server is running
    bool isRunning() const { return running_; }

    // Get server port
    int getPort() const { return port_; }

    // Set log callback
    void setLogCallback(LogCallback callback);

private:
    PercipioTcpLogServer();
    ~PercipioTcpLogServer();

    void run();
    void handleClient(int client_fd, const std::string& client_ip, int client_port);
    void setNonBlocking(int fd);
    //LogLevel parseLogLevel(const std::string& message);
    //std::string getLevelString(LogLevel level);
    //void consoleOutput(LogLevel level, const std::string& message, const std::string& client_ip, int client_port);
    void consoleOutput(const std::string& message, const std::string& client_ip, int client_port);

private:
    int server_fd_;
    int port_;
    std::atomic<bool> running_;
    std::thread server_thread_;
    std::mutex callback_mutex_;
    LogCallback log_callback_;
    std::vector<std::thread> client_threads_;
    std::mutex thread_mutex_;
};

#endif //_PERCIPIO_LOG_SERVER_H_