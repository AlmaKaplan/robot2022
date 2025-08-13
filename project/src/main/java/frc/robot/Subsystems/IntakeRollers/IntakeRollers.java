
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
  private static IntakeRollers rollers;

  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;

  private final DigitalInput sensor;

  private final StatusSignal<Current> current;
  private final StatusSignal<Voltage> volts;
  private final StatusSignal<AngularVelocity> velocity;

  
  private IntakeRollers() {
    motor = new TalonFX(PortMap.IntakeRollersPorts.INTAKE_MOTOR);
    motorConfig = new TalonFXConfiguration();

    sensor = new DigitalInput(PortMap.IntakeRollersPorts.INTAKE_ARM_DIGITAL_INPUT_SENSOR);

    current = motor.getStatorCurrent();
    volts = motor.getMotorVoltage();
    velocity = motor.getVelocity();

    config();
  }

  private void config() {
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

  public static IntakeRollers getInstance() {
    if(rollers == null) {
      rollers = new IntakeRollers();
    }
    return rollers;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(volts, current, velocity);
  }
}
