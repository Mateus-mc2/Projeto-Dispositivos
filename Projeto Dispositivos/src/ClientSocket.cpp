#include "ClientSocket.h"

namespace connection {

void ClientSocket::Close() {
  closesocket(this->connection_socket_);
  WSACleanup();
  this->closed_ = true;
}

void ClientSocket::Connect() {
  this->connection_socket_ = INVALID_SOCKET;

  AddrInfo *result = nullptr;
  AddrInfo hints;

  int flag;

  if (flag = WSAStartup(MAKEWORD(2, 2), &this->data_)) {
    throw SocketException("WSAStartup failed with error " + std::to_string(flag) + ".");
  }

  ZeroMemory(&hints, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  if (flag = getaddrinfo(this->host_.c_str(), this->port_.c_str(), &hints, &result)) {
    WSACleanup();
    throw SocketException("Function getaddrinfo failed with error " + std::to_string(flag) + ".");
  }

  bool connection_established = false;

  for (AddrInfo *ptr = result; ptr && !connection_established; ptr = ptr->ai_next) {
    this->connection_socket_ = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

    if (this->connection_socket_ == INVALID_SOCKET) {
      std::string error = std::to_string(WSAGetLastError());
      freeaddrinfo(result);
      WSACleanup();
      throw SocketException("Failed to create client socket (error " + error + ").");
    }

    flag = connect(this->connection_socket_, ptr->ai_addr, static_cast<int>(ptr->ai_addrlen));

    if (flag == SOCKET_ERROR) {
      closesocket(this->connection_socket_);
      this->connection_socket_ = INVALID_SOCKET;
    } else {
      connection_established = true;
    }
  }

  freeaddrinfo(result);

  if (!connection_established) {
    // Same as connection_socket == INVALID_SOCKET.
    WSACleanup();
    throw SocketException("Unable to connect to server.");
  }
}

void ClientSocket::Send(const std::string &message) {
  int num_bytes_sent;
  num_bytes_sent = send(this->connection_socket_, message.c_str(),
    static_cast<int>(message.size()), 0);

  if (num_bytes_sent == SOCKET_ERROR) {
    std::string error = std::to_string(WSAGetLastError());
    closesocket(this->connection_socket_);
    WSACleanup();
    throw SocketException("Failed to send message to client (error " + error + ").");
  }

  this->Shutdown(connection::Mode::kSend);
}


std::string ClientSocket::Receive() {
  std::string message;
  int num_bytes_received;

  do {
    this->buffer_.fill(0);
    num_bytes_received = recv(this->connection_socket_, this->buffer_.data(),
                              kDefaultBufferLength, 0);

    if (num_bytes_received == SOCKET_ERROR) {
      std::string error = std::to_string(WSAGetLastError());
      closesocket(this->connection_socket_);
      WSACleanup();
      throw SocketException("Failed to receive message from client (error " + error + ").");
    }
    else if (num_bytes_received > 0) {
      message += std::string(this->buffer_.data());
    }
  } while (num_bytes_received > 0);

  return message;
}

void ClientSocket::Shutdown(const Mode &mode) {
  int flag;

  switch (mode) {
    case Mode::kSend:
      flag = shutdown(this->connection_socket_, SD_SEND);
      break;

    case Mode::kReceive:
      flag = shutdown(this->connection_socket_, SD_RECEIVE);
      break;

    default:  // Mode::kBoth;
      flag = shutdown(this->connection_socket_, SD_BOTH);
      break;
  }

  if (flag == SOCKET_ERROR) {
    std::string error = std::to_string(WSAGetLastError());
    closesocket(this->connection_socket_);
    WSACleanup();
    throw SocketException("Failed to shutdown client socket (error " + error + ").");
  }
}

}  // namespace connection

