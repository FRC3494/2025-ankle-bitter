package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final AutoFactory autoFactory;

  private final Drive drive;

  public Autos(Drive drive) {
    this.drive = drive;

    autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            false,
            drive,
            this::logTrajectory);
  }

  private void logTrajectory(Trajectory<SwerveSample> traj, boolean finishing) {
    Logger.recordOutput("Odometry/ChoreoTraj", traj.getPoses());
  }

  public Command odometryTest2Cmd() {
    return Commands.sequence(
        autoFactory.resetOdometry("Odometry Test 2"), autoFactory.trajectoryCmd("Odometry Test 2"));
  }

  public AutoRoutine odometryTest() {
    AutoRoutine routine = autoFactory.newRoutine("Odometry Test");

    AutoTrajectory traj = routine.trajectory("Odometry Test");

    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }

  public AutoRoutine odometryTest2() {
    AutoRoutine routine = autoFactory.newRoutine("Odometry Test 2");

    AutoTrajectory traj = routine.trajectory("Odometry Test 2");

    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    traj.done().onTrue(Commands.print("Traj done"));

    return routine;
  }
}
