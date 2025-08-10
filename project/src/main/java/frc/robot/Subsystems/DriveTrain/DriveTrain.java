
package frc.robot.Subsystems.DriveTrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.Subsystems.Shooter.ShooterConstants;

public class DriveTrain extends SubsystemBase {
  private TalonFX leftMotor;
  private TalonFXConfiguration leftMotorConfig;

  private TalonFX rightMotor;
  private TalonFXConfiguration rightMotorConfig;

  private StatusSignal<Current> leftCurrent;
  private StatusSignal<Voltage> leftVolts;
  private StatusSignal<AngularVelocity> leftVelocity;

  private StatusSignal<Current> rightCurrent;
  private StatusSignal<Voltage> rightVolts;
  private StatusSignal<AngularVelocity> rightVelocity;

  private final VelocityVoltage rightPIDVelocity;
  private final VelocityVoltage leftPIDVelocity;

  public DriveTrain() {
    leftMotor = new TalonFX(PortMap.DriveTrain.DRIVE_TRAIN_LEFT_MOTOR);
    leftMotorConfig = new TalonFXConfiguration();

    rightMotor = new TalonFX(PortMap.DriveTrain.DRIVE_TRAIN_RIGHT_MOTOR);
    rightMotorConfig = new TalonFXConfiguration();

    leftCurrent = leftMotor.getStatorCurrent();
    leftVolts = leftMotor.getMotorVoltage();
    leftVelocity = leftMotor.getVelocity();

    rightCurrent = rightMotor.getStatorCurrent();
    rightVolts = rightMotor.getMotorVoltage();
    rightVelocity = rightMotor.getVelocity();

    rightPIDVelocity = new VelocityVoltage(0);
    leftPIDVelocity = new VelocityVoltage(0);

    leftConfig();
    rightConfig();
  }

  public void leftConfig() {
    leftMotorConfig.Feedback.RotorToSensorRatio = DriveTrainConstants.GEAR;

    leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leftMotorConfig.Slot0.kP = DriveTrainConstants.left_kP;
    leftMotorConfig.Slot0.kI = DriveTrainConstants.left_kI;
    leftMotorConfig.Slot0.kD = DriveTrainConstants.left_kD;

    leftMotor.getConfigurator().apply(leftMotorConfig);
  }

  public void rightConfig() {
    rightMotorConfig.Feedback.RotorToSensorRatio = DriveTrainConstants.GEAR;

    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rightMotorConfig.Slot0.kP = DriveTrainConstants.right_kP;
    rightMotorConfig.Slot0.kI = DriveTrainConstants.right_kI;
    rightMotorConfig.Slot0.kD = DriveTrainConstants.right_kD;

    rightMotor.getConfigurator().apply(rightMotorConfig);
  }

  public void leftControl(double SetPoint) {
    double ff= (SetPoint / (6000 / ShooterConstants.GEAR)) * 12;

    leftMotor.setControl(leftPIDVelocity.withVelocity(SetPoint).withSlot(0).withFeedForward(ff));
  }

  public void rightControl(double SetPoint) {
    double ff= (SetPoint / (6000 / ShooterConstants.GEAR)) * 12;

    rightMotor.setControl(rightPIDVelocity.withVelocity(SetPoint).withSlot(0).withFeedForward(ff));
  }

  public double getRightVelocity() {
    return rightVelocity.getValueAsDouble()*60;
  }

  public double getRightCurrent() {
    return rightCurrent.getValueAsDouble();
  }

  public double getRightVoltage() {
    return rightVolts.getValueAsDouble();
  }

  public double getLeftVelocity() {
    return leftVelocity.getValueAsDouble()*60;
  }

  public double getLeftCurrent() {
    return leftCurrent.getValueAsDouble();
  }

  public double getLeftVoltage() {
    return leftVolts.getValueAsDouble();
  }


  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(leftVolts, leftCurrent, leftVelocity, rightCurrent, rightVelocity, rightVolts);
  }
}

