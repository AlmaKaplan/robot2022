
package frc.robot.Subsystems.Transfer;

import com.MAutils.DashBoard.DashBoard;
import com.MAutils.DashBoard.DashBoardTab;
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
  private static Transfer transfer;

  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;

  private final DigitalInput firstSensor;
  private final DigitalInput secondSensor;

  private final StatusSignal<Current> current;
  private final StatusSignal<Voltage> volts;
  private final StatusSignal<AngularVelocity> velocity;

  private final DashBoardTab transferTab;

  private double ballInTransfer;

  private Transfer() {
    motor = new TalonFX(PortMap.TransferPorts.TRANSFER_MOTOR);
    motorConfig = new TalonFXConfiguration();

    firstSensor = new DigitalInput(PortMap.TransferPorts.TRANSFER_FIRST_DIGITAL_INPUT_SENSOR);
    secondSensor = new DigitalInput(PortMap.TransferPorts.TRANSFER_SECOND_DIGITAL_INPUT_SENSOR);

    current = motor.getStatorCurrent();
    volts = motor.getMotorVoltage();
    velocity = motor.getVelocity();
    
    transferTab = new DashBoardTab("transfer");

    ballInTransfer = 0;

    config();
  }

  private void config() {
    motorConfig.Feedback.RotorToSensorRatio = TransferConstants.GEAR;

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(motorConfig);
  }

  public void setVoltage(double volt) {
    motor.setVoltage(volt);
  }

  public boolean isFirstSensor() {
    return firstSensor.get();
  }

  public boolean isSeconSensor() {
    return secondSensor.get();
  }

  public boolean isTwoSensors() {
    return isFirstSensor() && isSeconSensor();
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

  public double ballinTransfer() {
    if (isFirstSensor()) {
      ballInTransfer ++;
    }
    if (isSeconSensor()) {
      ballInTransfer ++;
    }
    if (!isTwoSensors()) {
      ballInTransfer = 0;
    }
    return ballInTransfer;
  }

  public static Transfer getInstance() {
    if (transfer == null ) {
      transfer = new Transfer();
    }
    return transfer;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(current, volts, velocity);
    transferTab.addNum("Ball in transfer", ballInTransfer);
  }
}
