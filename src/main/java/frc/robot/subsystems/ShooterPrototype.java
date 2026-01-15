package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterPrototype extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;

  private double targetSpeed = 0;

  public ShooterPrototype() {
    leftMotor = new SparkMax(24, MotorType.kBrushless);
    rightMotor = new SparkMax(25, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kCoast);
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kCoast).inverted(true).follow(leftMotor);
    rightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("ShooterPrototype/LeftSpeedRPM", leftMotor.getEncoder().getVelocity());
    Logger.recordOutput("ShooterPrototype/RightSpeedRPM", rightMotor.getEncoder().getVelocity());
  }

  public void setTargetSpeed(double speed) {
    targetSpeed = MathUtil.clamp(speed, -1.0, 1.0);

    leftMotor.set(speed);
  }

  public Command speedUp() {
    return Commands.runOnce(
        () -> {
          targetSpeed += 0.1;
          setTargetSpeed(targetSpeed);
        });
  }

  public Command speedDown() {
    return Commands.runOnce(
        () -> {
          targetSpeed -= 0.1;
          setTargetSpeed(targetSpeed);
        });
  }
}
