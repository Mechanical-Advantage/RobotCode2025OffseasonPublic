package test;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import test.InputMessage;
import test.OutputMessage;
import org.littletonrobotics.junction.Logger;

public class SocketTestServer {
  private static InputMessage inputs = InputMessage.getDefaultInstance();
  private static OutputMessage.Builder outputsBuilder = OutputMessage.newBuilder();

  private static DatagramSocket socket;
  private static InetAddress clientAddress;
  private static int clientPort;
  private static boolean lastConnected = false;

  private static double lastMessageTimestamp = -1.0;
  private static int incomingBytes = 0;
  private static int outgoingBytes = 0;
  private static Timer bandwidthTimer = new Timer(); // TODO fix

  private static final Lock inputsLock = new ReentrantLock();
  private static final Condition newInputsCondition = inputsLock.newCondition();

  public static void start() {
    if (socket != null)
      return;

    try {
      socket = new DatagramSocket(Constants.serverPort);
    } catch (SocketException e) {
      e.printStackTrace();
    }

    if (socket != null) {
      var serverThread = new Thread(
          () -> {
            System.out.println(
                "[SocketTestServer] Socket ready on port "
                    + Integer.toString(Constants.serverPort)
                    + ", waiting for client...");
            byte[] buffer = new byte[Constants.maxPacketSizeBytes];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            while (true) {
              try {
                socket.receive(packet);

                if (clientAddress == null
                    || !clientAddress.equals(packet.getAddress())
                    || clientPort != packet.getPort()) {
                  System.out.println(
                      "[SocketTestServer] Connected to client "
                          + packet.getAddress().getHostAddress()
                          + ":"
                          + packet.getPort());
                  bandwidthTimer.reset();
                }

                clientAddress = packet.getAddress();
                clientPort = packet.getPort();

                var newInputs = InputMessage.parseFrom(
                    ByteBuffer.wrap(packet.getData(), 0, packet.getLength()));

                inputsLock.lock();
                try {
                  inputs = newInputs;
                  lastMessageTimestamp = Timer.getFPGATimestamp();
                  incomingBytes += packet.getLength();
                  newInputsCondition.signalAll();
                } finally {
                  inputsLock.unlock();
                }
              } catch (IOException e) {
                clientAddress = null;
                System.out.println(
                    "[SocketTestServer] Error receiving packet, waiting for reconnection...");
                e.printStackTrace();
              }
            }
          });

      serverThread.setName("SocketTestServer");
      serverThread.setDaemon(true);
      serverThread.start();
      bandwidthTimer.start();
    }
  }

  public static boolean waitForInputMessage(long timeoutUs) {
    long timeoutNanos = (long) (timeoutUs * 1000L);
    inputsLock.lock();
    try {
      double initialTimestamp = lastMessageTimestamp;
      while (lastMessageTimestamp == initialTimestamp) {
        if (timeoutNanos <= 0L) {
          return false;
        }
        timeoutNanos = newInputsCondition.awaitNanos(timeoutNanos);
      }
      return true; // New message was received
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      return false;
    } finally {
      inputsLock.unlock();
    }
  }

  public static boolean isConnected() {
    return clientAddress != null
        && Timer.getFPGATimestamp() - lastMessageTimestamp < (double) Constants.timeoutMs / 1000.0;
  }

  public static InputMessage getInputMessage() {
    inputsLock.lock();
    try {
      return inputs;
    } finally {
      inputsLock.unlock();
    }
  }

  public static void setOutputMessage(String message) {
    outputsBuilder.setMessage(message);
  }

  public static void processIncoming() {
    inputsLock.lock();
    try {
      outputsBuilder.setSeqnum(inputs.getSeqnum());
      if (bandwidthTimer.advanceIfElapsed(1)) {
        System.out.println("InputsMbps: " + (incomingBytes * 8.0 / 1.0e6));
        System.out.println("OutputsMbps: " + (outgoingBytes * 8.0 / 1.0e6));
        incomingBytes = 0;
        outgoingBytes = 0;
      }
    } finally {
      inputsLock.unlock();
    }
  }

  public static void processOutgoing() {
    if (!isConnected()) {
      if (lastConnected) {
        System.out.println("[SocketTestServer] Client timed out, waiting for reconnection...");
      }
      lastConnected = false;
      return;
    }
    lastConnected = true;

    try {
      byte[] payload = outputsBuilder.build().toByteArray();
      DatagramPacket packet = new DatagramPacket(payload, payload.length, clientAddress, clientPort);
      socket.send(packet);
      outgoingBytes += payload.length;
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private SocketTestServer() {
  }
}
