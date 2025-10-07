package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class OI {
  private static EventLoop eventLoop = new EventLoop();
  private static XboxController primaryController = new XboxController(0);

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

  public static BooleanEvent downUpMiddle() {
    return primaryController.x(eventLoop);
  }

  public static void update() {
    eventLoop.poll();
  }
}
