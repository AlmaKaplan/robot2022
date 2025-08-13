
package frc.robot.Subsystems.Shooter;

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

public class Shooter extends SubsystemBase {
  private static Shooter shooter;

  private final TalonFX leftMotor;
  private final TalonFXConfiguration leftMotorConfig;

  private final TalonFX rightMotor;
  private final TalonFXConfiguration rightMotorConfig;

  private final StatusSignal<Current> leftCurrent;
  private final StatusSignal<Voltage> leftVolts;
  private final StatusSignal<AngularVelocity> leftVelocity;

  private final StatusSignal<Current> rightCurrent;
  private final StatusSignal<Voltage> rightVolts;
  private final StatusSignal<AngularVelocity> rightVelocity;

  private final VelocityVoltage velocity;

  private Shooter() {
    leftMotor = new TalonFX(PortMap.ShooterPorts.SHOOTER_LEFT_MOTOR);
    leftMotorConfig = new TalonFXConfiguration();

    rightMotor = new TalonFX(PortMap.ShooterPorts.SHOOTER_RIGHT_MOTOR);
    rightMotorConfig = new TalonFXConfiguration();

    leftCurrent = leftMotor.getStatorCurrent();
    leftVolts = leftMotor.getMotorVoltage();
    leftVelocity = leftMotor.getVelocity();

    rightCurrent = rightMotor.getStatorCurrent();
    rightVolts = rightMotor.getMotorVoltage();
    rightVelocity = rightMotor.getVelocity();

    velocity = new VelocityVoltage(0);

    leftConfig();
    rightConfig();
  }

  private void leftConfig() {
    leftMotorConfig.Feedback.RotorToSensorRatio = ShooterConstants.GEAR;

    leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leftMotorConfig.Slot0.kP = ShooterConstants.left_kP;
    leftMotorConfig.Slot0.kI = ShooterConstants.left_kI;
    leftMotorConfig.Slot0.kD = ShooterConstants.left_kD;

    leftMotor.getConfigurator().apply(leftMotorConfig);
  }

  private void rightConfig() {
    rightMotorConfig.Feedback.RotorToSensorRatio = ShooterConstants.GEAR;

    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rightMotorConfig.Slot0.kP = ShooterConstants.right_kP;
    rightMotorConfig.Slot0.kI = ShooterConstants.right_kI;
    rightMotorConfig.Slot0.kD = ShooterConstants.right_kD;

    rightMotor.getConfigurator().apply(rightMotorConfig);
  }


  public void setVelocity(double SetPoint) {
    double ff= (SetPoint / (6000 / ShooterConstants.GEAR)) * 12;

    rightMotor.setControl(velocity.withVelocity(SetPoint).withSlot(0).withFeedForward(ff));
    leftMotor.setControl(velocity.withVelocity(SetPoint).withSlot(0).withFeedForward(ff));
  }

  public void setVoltage(double volt) {
    leftMotor.setVoltage(volt);
    rightMotor.setVoltage(volt);
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

  public static Shooter getInstance() {
    if(shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  @Override
  public void periodic() {
      BaseStatusSignal.refreshAll(leftVolts, leftCurrent, leftVelocity, rightCurrent, rightVelocity, rightVolts);
  }
}
