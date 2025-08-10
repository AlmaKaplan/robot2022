
package frc.robot.Subsystems.IntakeArm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class IntakeArm extends SubsystemBase {

  private TalonFX motor;
  private TalonFXConfiguration motorConfig;

  private StatusSignal<Current> current;
  private StatusSignal<Voltage> volts;
  private StatusSignal<Angle> position;

  private PositionVoltage control;


  public IntakeArm() {

    motor = new TalonFX(PortMap.IntakeArm.INTAKE_ARM_MOTOR);
    motorConfig = new TalonFXConfiguration();

    current = motor.getStatorCurrent();
    volts = motor.getMotorVoltage();
    position = motor.getPosition();

    control = new PositionVoltage(0);

    config();
  }

  public void config() {
    motorConfig.Feedback.RotorToSensorRatio = IntakeArmConstants.GEAR;

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motorConfig.Slot0.kP = IntakeArmConstants.kP;
    motorConfig.Slot0.kP = IntakeArmConstants.kI;
    motorConfig.Slot0.kP = IntakeArmConstants.kD;

    motor.getConfigurator().apply(motorConfig);
  }

  public void setControl(double setPoint) {
    motor.setControl(control.withPosition(setPoint).withSlot(0));
  }

  public double getPosition() {
    return position.getValueAsDouble() *360;
  }

  public double getCurrent() {
    return current.getValueAsDouble();
  }

  public double getVoltage() {
    return volts.getValueAsDouble();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(volts, current, position);
  }
}
