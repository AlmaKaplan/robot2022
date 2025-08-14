
package frc.robot;



import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Command.IntakeAndShooter;
import frc.robot.Command.IntakeCommand;
import frc.robot.Command.ShooterCommand;
import frc.robot.Subsystems.Transfer.Transfer;

public class RobotContainer {
  private static final PS5Controller controller = new PS5Controller(PortMap.ControllerPorts.CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    new Trigger(() ->  controller.getTriangleButton() && !Transfer.getInstance().isTwoSensors()).onTrue(new IntakeCommand());

    new Trigger(() -> controller.getCircleButton() &&
     (Transfer.getInstance().isFirstSensor() || Transfer.getInstance().isSeconSensor())).onTrue(new ShooterCommand());
  
    new Trigger(() -> controller.getL1Button()).whileTrue(new IntakeAndShooter());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static double getLeftY() {
    return controller.getLeftY();
  }

  public static double getRightY() {
    return controller.getLeftY();
  }
}
