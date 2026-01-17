package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

public class PheonixOdometryThread extends Thread {
  private static PheonixOdometryThread instance = null;
  private final List<BaseStatusSignal> pheonixSignals = new ArrayList<>();
  private final List<Queue<Double>> pheonixQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private PheonixOdometryThread() {
    setName("PheonixOdometryThread");
    setDaemon(true);
  }

  public static PheonixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PheonixOdometryThread();
    }
    return instance;
  }

  @Override
  public void start() {
    super.start();
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> registerSignal(BaseStatusSignal signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      pheonixSignals.add(signal);
      pheonixQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      Drive.odometryLock.lock();
      try {
        double timestamp = Timer.getFPGATimestamp() / 1e6;
        double latency = 0.0;
        for (var signal : pheonixSignals) {
          latency += signal.getTimestamp().getLatency();
        }
        timestamp -= latency / pheonixSignals.size();

        for (int i = 0; i < pheonixSignals.size(); ++i) {
          pheonixQueues.get(i).offer(pheonixSignals.get(i).getValueAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); ++i) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
