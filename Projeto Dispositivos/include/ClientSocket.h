#ifndef CLIENT_SOCKET_H_
#define CLIENT_SOCKET_H_


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

enum class Mode { kSend, kReceive, kBoth };

class SocketException : public std::exception {
public:
  explicit SocketException(const std::string &error) : kErrorMsg(error) {}

  const char* what() const { return kErrorMsg.c_str(); }

private:
  const std::string kErrorMsg;
};

class ClientSocket {
 public:
  explicit ClientSocket(const std::string &server_host, const std::string &port) :
      host_(server_host), port_(port), closed_(false) {}
  ~ClientSocket() {}

  void Connect();
  void Send(const std::string &message);
  std::string Receive();
  void Shutdown(const Mode &mode);
  void Close();

  inline bool closed() const { return this->closed_; }

 private:
  typedef struct addrinfo AddrInfo;

  WSAData data_;
  SOCKET connection_socket_;
  std::string host_;
  std::string port_;
  std::array<char, kDefaultBufferLength> buffer_;
  bool closed_;
};


}  // namespace connection

#endif  // CLIENT_SOCKET_H_
