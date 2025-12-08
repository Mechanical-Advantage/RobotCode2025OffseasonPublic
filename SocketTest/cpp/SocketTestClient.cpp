#include "SocketTestClient.h"

#include <arpa/inet.h>
#include <chrono>
#include <google/protobuf/util/message_differencer.h>
#include <hal/DriverStation.h>
#include <messages.pb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "Constants.h"

#include <endian.h>

bool SocketTestClient::started = false;
int SocketTestClient::seqnum = 0;
std::thread SocketTestClient::clientThread;
std::atomic<double> SocketTestClient::lastPacketTimestamp{-1.0};
int SocketTestClient::lastPacketSeqnum = 0;
int SocketTestClient::packetLossCount = 0;
int SocketTestClient::packetLossFirstSeqnum = 0;
int SocketTestClient::clientSocketFd = -1;
struct sockaddr_in SocketTestClient::serverAddress;
bool SocketTestClient::lastConnected = false;

std::mutex SocketTestClient::socketAccessMutex;
std::mutex SocketTestClient::outputsAccessMutex;
std::mutex SocketTestClient::inputsAccessMutex;

test::InputMessage SocketTestClient::inputs = test::InputMessage();
test::OutputMessage SocketTestClient::outputs = test::OutputMessage();
test::OutputMessage SocketTestClient::newOutputs = test::OutputMessage();

void SocketTestClient::runClient(std::function<void()> newMessage) {
  std::vector<uint8_t> receiveBuffer(Constants::maxPacketSizeBytes);

  struct timeval timeoutSetting;
  timeoutSetting.tv_sec = 1;
  timeoutSetting.tv_usec = 0;
  if (setsockopt(clientSocketFd, SOL_SOCKET, SO_RCVTIMEO, &timeoutSetting,
                 sizeof(timeoutSetting)) < 0) {
    perror("[SocketTestClient] setsockopt SO_RCVTIMEO failed");
    return;
  }

  std::cout << "[SocketTestClient] Socket ready, waiting for server..."
            << std::endl;

  while (true) {
    ssize_t bytesReceived = recvfrom(clientSocketFd, receiveBuffer.data(),
                                     receiveBuffer.size(), 0, nullptr, nullptr);

    if (bytesReceived > 0) {
      if (!newOutputs.ParseFromArray(receiveBuffer.data(), bytesReceived)) {
        std::cerr << "[SocketTestClient] Failed to parse outputs message"
                  << std::endl;
        continue;
      }

      if (!isConnected()) {
        packetLossCount = 0;
        packetLossFirstSeqnum = newOutputs.seqnum();
      } else if (newOutputs.seqnum() > lastPacketSeqnum) {
        packetLossCount += newOutputs.seqnum() - lastPacketSeqnum - 1;
      }
      lastPacketSeqnum = newOutputs.seqnum();
      int packetCountTotal = newOutputs.seqnum() - packetLossFirstSeqnum + 1;
      if (packetCountTotal > 100) {
        std::cout << "Packet Loss: "
                  << (double)packetLossCount / packetCountTotal << std::endl;
      }

      std::string outputsStr{
          reinterpret_cast<const char *>(receiveBuffer.data()),
          (size_t)bytesReceived};

      lastPacketTimestamp.store(
          std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count());

      {
        std::lock_guard<std::mutex> lock(outputsAccessMutex);
        outputs = newOutputs;
      }

      newMessage();
    } else if (bytesReceived < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("[SocketTestClient] Error receiving packet");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }
}

void SocketTestClient::start(std::function<void()> newMessage) {
  if (started)
    return;
  started = true;

  clientSocketFd = socket(AF_INET, SOCK_DGRAM, 0);
  if (clientSocketFd < 0) {
    perror("[SocketTestClient] FATAL: Socket creation failed");
    return;
  }

  serverAddress.sin_family = AF_INET;
  serverAddress.sin_port = htons(Constants::serverPort);
  if (inet_pton(AF_INET, std::string(Constants::serverAddress).c_str(),
                &serverAddress.sin_addr) <= 0) {
    perror("[SocketTestClient] FATAL: Invalid server address");
    close(clientSocketFd);
    clientSocketFd = -1;
    return;
  }

  clientThread = std::thread(runClient, newMessage);
  if (clientThread.joinable()) {
    clientThread.detach();
  }
}

bool SocketTestClient::isConnected() {
  if (!started || lastPacketTimestamp.load() < 0) {
    return false;
  }
  return (std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count() -
          lastPacketTimestamp.load()) <
         (static_cast<double>(Constants::timeoutMs) / 1000.0);
}

void SocketTestClient::setInputs(test::InputMessage &newInputs) {
  std::lock_guard<std::mutex> lock(inputsAccessMutex);
  inputs = newInputs;
}

const test::OutputMessage SocketTestClient::getOutputs(bool &success) {
  std::lock_guard<std::mutex> lock(outputsAccessMutex);
  success = true;
  return outputs;
}

void SocketTestClient::processOutgoing() {
  std::lock_guard<std::mutex> lock(inputsAccessMutex);

  const bool connected = isConnected();
  if (connected && !lastConnected) {
    std::cout << "[SocketTestClient] Connected to server "
              << Constants::serverAddress << ":" << Constants::serverPort
              << std::endl;
  } else if (!connected && lastConnected) {
    std::cout << "[SocketTestClient] Server timed out, waiting for reconnection..."
              << std::endl;
  }
  lastConnected = connected;

  std::string serializedInputData;
  if (!inputs.SerializeToString(&serializedInputData)) {
    std::cerr << "[SocketTestClient] Failed to serialize inputs message"
              << std::endl;
    return;
  }

  std::lock_guard<std::mutex> socketLock(socketAccessMutex);
  if (clientSocketFd != -1) {
    ssize_t bytesSent =
        sendto(clientSocketFd, serializedInputData.data(),
               serializedInputData.length(), 0,
               (struct sockaddr *)&serverAddress, sizeof(serverAddress));
    if (bytesSent < 0) {
      perror("[SocketTestClient] Error sending packet");
    }
  }
}
