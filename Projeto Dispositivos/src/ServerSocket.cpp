#include "ServerSocket.h"


namespace connection {

ServerSocket::ServerSocket(const std::string &port)
  :  port_(port),
     welcome_socket_(INVALID_SOCKET),
     connection_socket_(INVALID_SOCKET),
     closed_(false) {
  AddrInfo *result = nullptr;
  AddrInfo hints;
  int flag;

  if (flag = WSAStartup(MAKEWORD(2, 2), &this->data)) {
    throw SocketException("WSAStartup failed with error " + std::to_string(flag) + ".");
  }

  ZeroMemory(&hints, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  hints.ai_flags = AI_PASSIVE;

  // Resolve the server address and port
  if (flag = getaddrinfo(nullptr, this->port_.c_str(), &hints, &result)) {
    throw SocketException("Function getaddrinfo failed with error: " + std::to_string(flag) + ".");
  }

  // Create a SOCKET for connecting to server
  this->welcome_socket_ = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
  if (this->welcome_socket_ == INVALID_SOCKET) {
    freeaddrinfo(result);
    std::string error = std::to_string(WSAGetLastError());
    throw SocketException("Couldn't create connection socket (error " + error + ").");
  }

  // Setup the TCP listening socket
  flag = bind(this->welcome_socket_, result->ai_addr, (int)result->ai_addrlen);
  if (flag == SOCKET_ERROR) {
    freeaddrinfo(result);
    std::string error = std::to_string(WSAGetLastError());
    throw SocketException("Couldn't bind server socket (error " + error + ").");
  }

  freeaddrinfo(result);
}

ServerSocket::~ServerSocket() {
  this->Close();
}

void ServerSocket::Close() {
  closesocket(this->welcome_socket_);
  WSACleanup();
  this->closed_ = true;
}

void ServerSocket::WaitConnection() {
  if (listen(this->welcome_socket_, SOMAXCONN) == SOCKET_ERROR) {
    std::string error = std::to_string(WSAGetLastError());
    throw SocketException("Listen socket failed with error " + error + ").");
  }

  // Accept a client socket
  this->connection_socket_ = accept(this->welcome_socket_, nullptr, nullptr);
  if (this->connection_socket_ == INVALID_SOCKET) {
    std::string error = std::to_string(WSAGetLastError());
    throw SocketException("Failed to accept client socket (error " + error + ").");
  }
}

void ServerSocket::Send(const std::string &message) {
  int num_bytes_sent;

  do {
    num_bytes_sent = send(this->connection_socket_, message.c_str(),
                          static_cast<int>(message.size()), 0);

    if (num_bytes_sent == SOCKET_ERROR) {
      std::string error = std::to_string(WSAGetLastError());
      throw SocketException("Failed to send message to client (error " + error + ").");
    }
  } while (num_bytes_sent > 0);
}

std::string ServerSocket::Receive() {
  std::string message;
  int num_bytes_received;

  do {
    num_bytes_received = recv(this->connection_socket_, this->buffer_.data(),
                              kDefaultBufferLength, 0);

    if (num_bytes_received == SOCKET_ERROR) {
      std::string error = std::to_string(WSAGetLastError());
      throw SocketException("Failed to receive message from client (error " + error + ").");
    } else {
      message += std::string(this->buffer_.data());
    }
  } while (num_bytes_received > 0);

  return message;
}

}
