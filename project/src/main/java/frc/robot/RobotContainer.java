
package frc.robot;


import com.MAutils.DashBoard.DashBoardTab;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Command.DriveTrainCommand;
import frc.robot.Command.IntakeAndShooter;
import frc.robot.Command.IntakeCommand;
import frc.robot.Command.ShooterCommand;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Transfer.Transfer;

public class RobotContainer {
  private final PS5Controller controller;
  private DriveTrainCommand driveTrain;

  public static DashBoardTab currentAction;

  public RobotContainer() {
    controller = new PS5Controller(PortMap.ControllerPorts.CONTROLLER_PORT);
    currentAction = new DashBoardTab("currentAction");
    driveTrain = new DriveTrainCommand(() -> controller.getLeftY(), () -> controller.getRightY());
    CommandScheduler.getInstance().setDefaultCommand(DriveTrain.getInstance(), driveTrain);
    configureBindings();
  }
// controller.getTriangleButton() &&
  private void configureBindings() {
    new Trigger(() ->  !Transfer.getInstance().isTwoSensors()).onTrue(new IntakeCommand());

    new Trigger(() -> controller.getCircleButton() &&
     (Transfer.getInstance().isFirstSensor() || Transfer.getInstance().isSeconSensor())).onTrue(new ShooterCommand());
  
    new Trigger(() -> controller.getL1Button()).whileTrue(new IntakeAndShooter());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
