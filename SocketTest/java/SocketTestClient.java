package test;

import com.google.protobuf.InvalidProtocolBufferException;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import test.InputMessage;
import test.OutputMessage;

public class SocketTestClient {
  private static InputMessage.Builder inputsBuilder = InputMessage.newBuilder();
  private static OutputMessage outputs = OutputMessage.getDefaultInstance();

  private static DatagramSocket socket;
  private static InetAddress serverAddress;
  private static boolean lastConnected = false;
  private static long lastPacketTimestampNs = -1;

  private static final Lock inputsLock = new ReentrantLock();
  private static final Lock outputsLock = new ReentrantLock();

  public static void start() {
    if (socket != null)
      return;

    try {
      socket = new DatagramSocket();
      serverAddress = InetAddress.getByName(Constants.serverAddress);
    } catch (SocketException | UnknownHostException e) {
      e.printStackTrace();
      return;
    }

    var clientThread = new Thread(() -> {
      System.out.println("[SocketTestClient] Socket ready, waiting for server...");
      byte[] buffer = new byte[Constants.maxPacketSizeBytes];
      DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
      while (true) {
        try {
          socket.receive(packet);
          var newOutputs =
              OutputMessage.parseFrom(ByteBuffer.wrap(packet.getData(), 0, packet.getLength()));

          outputsLock.lock();
          try {
            outputs = newOutputs;
            lastPacketTimestampNs = System.nanoTime();
          } finally {
            outputsLock.unlock();
          }
        } catch (InvalidProtocolBufferException e) {
          System.err.println("[SocketTestClient] Failed to parse outputs message");
        } catch (IOException e) {
          System.err.println("[SocketTestClient] Error receiving packet");
        }
      }
    });

    clientThread.setName("SocketTestClient");
    clientThread.setDaemon(true);
    clientThread.start();
  }

  public static boolean isConnected() {
    if (lastPacketTimestampNs < 0) {
      return false;
    }
    long timeoutNs = (long) Constants.timeoutMs * 1_000_000L;
    return (System.nanoTime() - lastPacketTimestampNs) < timeoutNs;
  }

  public static void setInputMessage(String message) {
    inputsLock.lock();
    try {
      inputsBuilder.setMessage(message);
    } finally {
      inputsLock.unlock();
    }
  }

  public static OutputMessage getOutputMessage() {
    outputsLock.lock();
    try {
      return outputs;
    } finally {
      outputsLock.unlock();
    }
  }

  public static void processOutgoing() {
    boolean connected = isConnected();
    if (connected && !lastConnected) {
      System.out.println(
          "[SocketTestClient] Connected to server "
              + Constants.serverAddress
              + ":"
              + Constants.serverPort);
    } else if (!connected && lastConnected) {
      System.out.println("[SocketTestClient] Server timed out, waiting for reconnection...");
    }
    lastConnected = connected;

    inputsLock.lock();
    try {
      inputsBuilder.setSeqnum(inputsBuilder.getSeqnum() + 1);
      byte[] payload = inputsBuilder.build().toByteArray();
      DatagramPacket packet =
          new DatagramPacket(payload, payload.length, serverAddress, Constants.serverPort);
      socket.send(packet);
    } catch (IOException e) {
      e.printStackTrace();
    } finally {
      inputsLock.unlock();
    }
  }

  private SocketTestClient() {}
}