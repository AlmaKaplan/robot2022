
package frc.robot.Subsystems.Transfer;

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

public class Transfer extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration motorConfig;

  private DigitalInput firstSensor;
  private DigitalInput secondSensor;

  private StatusSignal<Current> current;
  private StatusSignal<Voltage> volts;
  private StatusSignal<AngularVelocity> velocity;

  public Transfer() {
    motor = new TalonFX(PortMap.Transfer.TRANSFER_MOTOR);
    motorConfig = new TalonFXConfiguration();

    firstSensor = new DigitalInput(PortMap.Transfer.TRANSFER_FIRST_DIGITAL_INPUT_SENSOR);
    secondSensor = new DigitalInput(PortMap.Transfer.TRANSFER_SECOND_DIGITAL_INPUT_SENSOR);

    current = motor.getStatorCurrent();
    volts = motor.getMotorVoltage();
    velocity = motor.getVelocity();

    config();
  }

  public void config() {
    motorConfig.Feedback.RotorToSensorRatio = TransferConstants.GEAR;

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motor.getConfigurator().apply(motorConfig);
  }

  public void setVoltage(double volt) {
    motor.setVoltage(volt);
  }

  public boolean getFirstSensor() {
    return firstSensor.get();
  }

  public boolean getSeconSensor() {
    return secondSensor.get();
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
    BaseStatusSignal.refreshAll(current, volts, velocity);
  }
}
