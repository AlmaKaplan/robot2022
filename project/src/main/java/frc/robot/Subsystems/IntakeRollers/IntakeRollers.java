
package frc.robot.Subsystems.IntakeRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class IntakeRollers extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration motorConfig;

  private DigitalInput sensor;

  private StatusSignal<Current> current;
  private StatusSignal<Voltage> volts;
  private StatusSignal<AngularVelocity> velocity;

  
  public IntakeRollers() {
    motor = new TalonFX(PortMap.IntakeRollers.INTAKE_MOTOR);
    motorConfig = new TalonFXConfiguration();

    sensor = new DigitalInput(PortMap.IntakeRollers.INTAKE_ARM_DIGITAL_INPUT_SENSOR);

    current = motor.getStatorCurrent();
    volts = motor.getMotorVoltage();
    velocity = motor.getVelocity();

    config();
  }

  public void config() {
    motorConfig.Feedback.RotorToSensorRatio = IntakeRollersConstants.GEAR;

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motor.getConfigurator().apply(motorConfig);
  }

  public void setVoltage(double volt) {
    motor.setVoltage(volt);
  }

  public boolean getSensor() {
    return sensor.get();
  }

  public double getVelocity() {
    return velocity.getValueAsDouble()*60;
  }

  public double getCurrent() {
    return current.getValueAsDouble();
  }

  public double getVoltage() {
    return volts.getValueAsDouble();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(volts, current, velocity);
  }
}
