#ifndef SERVER_SOCKET_H_
#define SERVER_SOCKET_H_

#ifdef UNICODE
#undef UNICODE
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <Windows.h>
#include <WinSock2.h>
#include <WS2tcpip.h>

#include <array>
#include <exception>
#include <string>

#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

namespace connection {

const int kDefaultBufferLength = 512;

class SocketException : public std::exception {
 public:
  explicit SocketException(const std::string &error) : kErrorMsg(error) {}

  const char* what() const { return kErrorMsg.c_str(); }

 private:
  const std::string kErrorMsg;
};

class ServerSocket {
 public:
  explicit ServerSocket(const std::string &port);
  ~ServerSocket();

  void WaitConnection();
  void Send(const std::string &message);
  std::string Receive();

  inline bool closed() const { return this->closed_; }
  inline SOCKET connection_socket() const { return this->connection_socket_; };
 protected:
  void Close();

 private:
  typedef struct addrinfo AddrInfo;

  WSAData data;
  SOCKET welcome_socket_;
  SOCKET connection_socket_;
  std::string port_;
  std::array<char, kDefaultBufferLength> buffer_;
  bool closed_;
};

}  // namespace connection

#endif  // SERVER_SOCKET_H_
