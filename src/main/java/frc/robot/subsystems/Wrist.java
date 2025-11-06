package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  public SparkMax wristMotor;
  public SparkMax spinMotor;

  public double spinPower = 0.0;
  public double targetPos = 0;

  private boolean softLimitsEnabled = true;

  public Wrist() {
    wristMotor = new SparkMax(Constants.Wrist.wristMotorID, MotorType.kBrushless);
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.closedLoop.pidf(0.5, 0, 0, .1).outputRange(-0.3, 0.3);
    // wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristMotor.configure(
        wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    spinMotor = new SparkMax(Constants.Wrist.spinMotorID, MotorType.kBrushless);
    SparkMaxConfig spinConfig = new SparkMaxConfig();
    spinConfig.idleMode(IdleMode.kBrake);
    spinMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    super.periodic();
    if (OI.spinInPower()) {
      // TODO Set this to be smth else
      spinMotor.set(Constants.Wrist.spinPowerIn);
    } else if (OI.spinOutPower()) {
      spinMotor.set(-Constants.Wrist.spinPowerOut);
    } else {
      spinMotor.set(0);
    }

    if (OI.wristDownPower() >= Constants.Wrist.wristPowerDeadband) {
      targetPos += OI.wristDownPower() * Constants.Wrist.wristPowerCoef;
    } else if (OI.wristUpPower() >= Constants.Wrist.wristPowerDeadband) {
      targetPos -= OI.wristUpPower() * Constants.Wrist.wristPowerCoef;
    } else if (OI.downPreset()) {
      targetPos = Constants.Wrist.downPreset;
    } else if (OI.upPreset()) {
      targetPos = Constants.Wrist.upPreset;
    }
    if (softLimitsEnabled) {
      targetPos = MathUtil.clamp(targetPos, 0, 30);
    }
    wristMotor.getClosedLoopController().setReference(targetPos, ControlType.kPosition);
    double error = targetPos - wristMotor.getEncoder().getPosition();
    double kP = 1;
    double PIDpower = error * kP;
    PIDpower = MathUtil.clamp(PIDpower, -0.2, 0.2);
    // wristMotor.set(PIDpower);

    Logger.recordOutput("Wrist/PivotPosition", getPosition());
    Logger.recordOutput("Wrist/PivotAbsPosition", wristMotor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Wrist/PivotPower", PIDpower);
    Logger.recordOutput("Wrist/PivotTargetPos", targetPos);
    Logger.recordOutput("Wrist/PivotCurrent", wristMotor.getOutputCurrent());
    Logger.recordOutput("Wrist/SpinSpeed", spinMotor.getAppliedOutput());
  }

  public void setTarget(double targetPos) {
    this.targetPos = targetPos;
  }

  public double getPosition() {
    return wristMotor.getEncoder().getPosition();
  }

  public void setSoftLimits(boolean enabled) {
    softLimitsEnabled = enabled;
  }

  public void rezeroWrist() {
    wristMotor.getEncoder().setPosition(0);
  }
}
