#pragma once

#include <arpa/inet.h>
#include <messages.pb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <thread>

#include "Robot.h"

class SocketTestClient {
public:
  SocketTestClient() = delete;

  static void start(std::function<void()> newMessage);

  static bool isConnected();

  static void setInputs(test::InputMessage &inputs);

  static const test::OutputMessage getOutputs(bool &success);

  static void processOutgoing();

private:
  static void runClient(std::function<void()> newMessage);

  static bool started;
  static int seqnum;
  static std::thread clientThread;
  static std::atomic<double> lastPacketTimestamp;
  static int lastPacketSeqnum;
  static int packetLossCount;
  static int packetLossFirstSeqnum;
  static int clientSocketFd;
  static struct sockaddr_in serverAddress;
  static bool lastConnected;

  static std::mutex socketAccessMutex;
  static std::mutex outputsAccessMutex;
  static std::mutex inputsAccessMutex;

  static test::InputMessage inputs;
  static test::OutputMessage outputs;
  static test::OutputMessage newOutputs;
};
