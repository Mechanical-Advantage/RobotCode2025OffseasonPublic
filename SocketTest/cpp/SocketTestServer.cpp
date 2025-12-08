#include "SocketTestServer.h"

#include <unistd.h>

#include <chrono>
#include <iostream>
#include <vector>

#include "Constants.h"

bool SocketTestServer::started = false;
int SocketTestServer::serverSocketFd = -1;
struct sockaddr_in SocketTestServer::clientAddress;
bool SocketTestServer::clientAddressValid = false;
bool SocketTestServer::lastConnected = false;
std::atomic<double> SocketTestServer::lastMessageTimestamp{-1.0};
std::mutex SocketTestServer::inputsMutex;
std::condition_variable SocketTestServer::newInputsCondition;
test::InputMessage SocketTestServer::inputs = test::InputMessage();
test::OutputMessage SocketTestServer::outputs = test::OutputMessage();

void SocketTestServer::runServer() {
  std::cout << "[SocketTestServer] Socket ready on port " << Constants::serverPort
            << ", waiting for client..." << std::endl;
  std::vector<uint8_t> buffer(Constants::maxPacketSizeBytes);
  struct sockaddr_in currentClientAddr;
  socklen_t clientAddrLen = sizeof(currentClientAddr);

  while (true) {
    ssize_t bytesReceived =
        recvfrom(serverSocketFd, buffer.data(), buffer.size(), 0,
                 (struct sockaddr *)&currentClientAddr, &clientAddrLen);

    if (bytesReceived > 0) {
      if (!clientAddressValid ||
          clientAddress.sin_addr.s_addr != currentClientAddr.sin_addr.s_addr ||
          clientAddress.sin_port != currentClientAddr.sin_port) {
        char clientIpStr[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(currentClientAddr.sin_addr), clientIpStr,
                  INET_ADDRSTRLEN);
        std::cout << "[SocketTestServer] Connected to client " << clientIpStr
                  << ":" << ntohs(currentClientAddr.sin_port) << std::endl;
      }
      clientAddress = currentClientAddr;
      clientAddressValid = true;

      test::InputMessage newInputs;
      if (newInputs.ParseFromArray(buffer.data(), bytesReceived)) {
        std::unique_lock<std::mutex> lock(inputsMutex);
        inputs = newInputs;
        lastMessageTimestamp.store(
            std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        lock.unlock();
        newInputsCondition.notify_all();
      } else {
        std::cerr << "[SocketTestServer] Failed to parse input message"
                  << std::endl;
      }
    } else if (bytesReceived < 0) {
      perror("[SocketTestServer] Error receiving packet");
      clientAddressValid = false;
    }
  }
}

void SocketTestServer::start() {
  if (started)
    return;
  started = true;

  serverSocketFd = socket(AF_INET, SOCK_DGRAM, 0);
  if (serverSocketFd < 0) {
    perror("[SocketTestServer] FATAL: Socket creation failed");
    return;
  }

  struct sockaddr_in serverAddr;
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = INADDR_ANY;
  serverAddr.sin_port = htons(Constants::serverPort);

  if (bind(serverSocketFd, (struct sockaddr *)&serverAddr,
           sizeof(serverAddr)) < 0) {
    perror("[SocketTestServer] FATAL: Bind failed");
    close(serverSocketFd);
    serverSocketFd = -1;
    return;
  }

  std::thread serverThread(runServer);
  if (serverThread.joinable()) {
    serverThread.detach();
  }
}

bool SocketTestServer::waitForInputMessage(long timeoutUs) {
  std::unique_lock<std::mutex> lock(inputsMutex);
  double initialTimestamp = lastMessageTimestamp.load();

  auto status = newInputsCondition.wait_for(
      lock, std::chrono::microseconds(timeoutUs),
      [&] { return lastMessageTimestamp.load() != initialTimestamp; });

  return status; // Returns true if new message was received
}

bool SocketTestServer::isConnected() {
  if (!clientAddressValid || lastMessageTimestamp.load() < 0) {
    return false;
  }
  double now = std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();
  return (now - lastMessageTimestamp.load()) <
         (static_cast<double>(Constants::timeoutMs) / 1000.0);
}

const test::InputMessage SocketTestServer::getInputMessage() {
  std::lock_guard<std::mutex> lock(inputsMutex);
  return inputs;
}

void SocketTestServer::setOutputMessage(const std::string &message) {
  outputs.set_message(message);
}

void SocketTestServer::processIncoming() {
  std::lock_guard<std::mutex> lock(inputsMutex);
  outputs.set_seqnum(inputs.seqnum());
}

void SocketTestServer::processOutgoing() {
  if (!isConnected()) {
    if (lastConnected) {
      std::cout
          << "[SocketTestServer] Client timed out, waiting for reconnection..."
          << std::endl;
    }
    lastConnected = false;
    return;
  }
  lastConnected = true;

  std::string serializedOutputData;
  if (!outputs.SerializeToString(&serializedOutputData)) {
    std::cerr << "[SocketTestServer] Failed to serialize outputs message"
              << std::endl;
    return;
  }

  ssize_t bytesSent =
      sendto(serverSocketFd, serializedOutputData.data(),
             serializedOutputData.length(), 0,
             (struct sockaddr *)&clientAddress, sizeof(clientAddress));
  if (bytesSent < 0) {
    perror("[SocketTestServer] Error sending packet");
  }
}