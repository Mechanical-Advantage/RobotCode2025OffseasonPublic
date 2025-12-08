#pragma once

#include <arpa/inet.h>
#include <messages.pb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

class SocketTestServer {
public:
  SocketTestServer() = delete;

  static void start();

  static bool waitForInputMessage(long timeoutUs);

  static bool isConnected();

  static const test::InputMessage getInputMessage();

  static void setOutputMessage(const std::string &message);

  static void processIncoming();

  static void processOutgoing();

private:
  static void runServer();

  static bool started;
  static int serverSocketFd;
  static struct sockaddr_in clientAddress;
  static bool clientAddressValid;
  static bool lastConnected;

  static std.atomic<double> lastMessageTimestamp;

  static std::mutex inputsMutex;
  static std::condition_variable newInputsCondition;

  static test::InputMessage inputs;
  static test::OutputMessage outputs;
};