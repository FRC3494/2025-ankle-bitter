package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class WristSetTarget extends Command {
  Wrist wrist;
  double targetPos;

  public WristSetTarget(Wrist wrist, double targetPos) {
    this.wrist = wrist;
    this.targetPos = targetPos;
  }

  @Override
  public void execute() {
    wrist.setTarget(targetPos);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(wrist.getPosition() - targetPos) < 0.01;
  }
}
