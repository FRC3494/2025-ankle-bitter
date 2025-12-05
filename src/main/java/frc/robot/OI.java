package frc.robot;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {
  private static EventLoop eventLoop = new EventLoop();
  private static CommandXboxController primaryController = new CommandXboxController(0);

  public static class Drive {
    public static Trigger rezeroGyro() {
      return primaryController.back(eventLoop);
    }

    public static Trigger rezeroSwerveTurn() {
      return primaryController.povLeft();
    }

    public static Trigger stopWithX() {
      return primaryController.x(eventLoop);
    }
  }

  public static boolean spinInPower() {
    return primaryController.rightBumper(eventLoop).getAsBoolean();
  }

  public static boolean spinOutPower() {
    return primaryController.leftBumper(eventLoop).getAsBoolean();
  }

  public static double wristDownPower() {
    return primaryController.getRightTriggerAxis() * 0.15;
  }

  public static double wristUpPower() {
    return primaryController.getLeftTriggerAxis() * 0.15;
  }

  public static boolean upPreset() {
    return primaryController.b(eventLoop).getAsBoolean();
  }

  public static boolean downPreset() {
    return primaryController.a(eventLoop).getAsBoolean();
  }

  public static Trigger rezeroWrist() {
    return primaryController.start(eventLoop);
  }

  public static void update() {
    eventLoop.poll();
  }
}
