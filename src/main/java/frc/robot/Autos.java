package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class Autos {
  private final AutoFactory autoFactory;

  private final Drive drive;

  public Autos(Drive drive) {
    this.drive = drive;

    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, false, drive);
  }

  public AutoRoutine odometryTest() {
    AutoRoutine routine = autoFactory.newRoutine("Odometry Test");

    AutoTrajectory traj = routine.trajectory("Odometry Test");

    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }
}
