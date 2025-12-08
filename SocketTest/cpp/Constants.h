#pragma once

#include <string_view>

constexpr int serverPort = 8000;
constexpr std::string_view serverAddress = "127.0.0.1";
constexpr int timeoutMs = 10000;
constexpr int maxPacketSizeBytes = 16384;