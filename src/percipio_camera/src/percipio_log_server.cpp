#include <ros/ros.h>
#include "percipio_camera/percipio_log_server.h"

PercipioTcpLogServer::PercipioTcpLogServer() 
    : server_fd_(-1)
    , port_(9000)
    , running_(false) {
}

PercipioTcpLogServer::~PercipioTcpLogServer() {
    stop();
}

bool PercipioTcpLogServer::start(int port) {
    if (running_) {
        ROS_ERROR("[ERROR] Server is already running on port %d", port_);
        return false;
    }

    port_ = port;

    // Create socket
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        ROS_ERROR("[ERROR] Failed to create socket: %s", strerror(errno));
        return false;
    }

    // Set socket options
    int opt = 1;
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        ROS_ERROR("[ERROR] Failed to set socket options: %s", strerror(errno));
        close(server_fd_);
        return false;
    }

    // Bind socket
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);

    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        ROS_ERROR("[ERROR] Failed to bind socket: %s", strerror(errno));
        close(server_fd_);
        return false;
    }

    // Listen for connections
    if (listen(server_fd_, 10) < 0) {
        ROS_ERROR("[ERROR] Failed to listen on socket: %s", strerror(errno));
        close(server_fd_);
        return false;
    }

    // Set socket to non-blocking mode
    setNonBlocking(server_fd_);

    // Start server thread
    running_ = true;
    server_thread_ = std::thread([this]() { this->run(); });

    ROS_INFO("[INFO] TCP Log Server started on port %d", port_ );
    return true;
}

void PercipioTcpLogServer::stop() {
    if (!running_) return;

    running_ = false;

    // Close server socket to interrupt accept()
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }

    // Wait for server thread to finish
    if (server_thread_.joinable()) {
        server_thread_.join();
    }

    // Wait for all client threads to finish
    {
        std::lock_guard<std::mutex> lock(thread_mutex_);
        for (auto& thread : client_threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        client_threads_.clear();
    }

    ROS_INFO("[INFO] TCP Log Server stopped");
}

void PercipioTcpLogServer::run() {
    struct pollfd poll_fds[1];
    poll_fds[0].fd = server_fd_;
    poll_fds[0].events = POLLIN;

    while (running_) {
        int poll_count = poll(poll_fds, 1, 100); // 100ms timeout

        if (poll_count < 0) {
            if (errno == EINTR) continue;
            ROS_ERROR("[ERROR] Poll error: %s", strerror(errno));
            break;
        }

        if (poll_count == 0) {
            // Timeout, continue polling
            continue;
        }

        if (poll_fds[0].revents & POLLIN) {
            // Accept new connection
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
            
            if (client_fd < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    continue;
                }
                ROS_ERROR("[ERROR] Failed to accept connection: %s", strerror(errno));
                continue;
            }

            // Get client info
            std::string client_ip = inet_ntoa(client_addr.sin_addr);
            int client_port = ntohs(client_addr.sin_port);

            ROS_INFO("[INFO] Client connected: %s:%d",client_ip.c_str(), client_port);

            // Set client socket to non-blocking
            setNonBlocking(client_fd);

            // Create thread to handle client
            std::thread client_thread([this, client_fd, client_ip, client_port]() {
                this->handleClient(client_fd, client_ip, client_port);
            });

            // Store thread reference
            {
                std::lock_guard<std::mutex> lock(thread_mutex_);
                client_threads_.push_back(std::move(client_thread));
            }
        }
    }

    // Clean up any remaining threads
    {
        std::lock_guard<std::mutex> lock(thread_mutex_);
        for (auto& thread : client_threads_) {
            if (thread.joinable()) {
                thread.detach();
            }
        }
        client_threads_.clear();
    }
}

void PercipioTcpLogServer::handleClient(int client_fd, const std::string& client_ip, int client_port) {
    char buffer[4096];
    std::string message_buffer;

    struct pollfd poll_fds[1];
    poll_fds[0].fd = client_fd;
    poll_fds[0].events = POLLIN;

    while (running_) {
        int poll_count = poll(poll_fds, 1, 100); // 100ms timeout

        if (poll_count < 0) {
            if (errno == EINTR) continue;
            break;
        }

        if (poll_count == 0) {
            // Timeout, continue
            continue;
        }

        if (poll_fds[0].revents & POLLIN) {
            int bytes_read = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
            
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                message_buffer.append(buffer);

                // Process complete messages (delimited by newline)
                size_t pos;
                while ((pos = message_buffer.find('\n')) != std::string::npos) {
                    std::string message = message_buffer.substr(0, pos);
                    message_buffer.erase(0, pos + 1);

                    if (!message.empty()) {
                        consoleOutput(message, client_ip, client_port);

                        // Call callback
                        std::lock_guard<std::mutex> lock(callback_mutex_);
                        if (log_callback_) {
                            log_callback_(message);
                        }
                    }
                }
            } else if (bytes_read == 0) {
                // Connection closed by client
                break;
            } else {
                if (errno != EWOULDBLOCK && errno != EAGAIN) {
                    // Real error
                    break;
                }
            }
        }
    }

    close(client_fd);
    ROS_INFO("[INFO] Client disconnected: %s:%d", client_ip.c_str(), client_port);
}

void PercipioTcpLogServer::setNonBlocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return;
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

void PercipioTcpLogServer::consoleOutput(const std::string& message, const std::string& client_ip, int client_port) {
    ROS_INFO("%s", message.c_str());
}

void PercipioTcpLogServer::setLogCallback(LogCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    log_callback_ = std::move(callback);
}