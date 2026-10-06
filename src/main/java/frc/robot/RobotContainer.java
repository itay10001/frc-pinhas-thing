package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MechanismCommands;
import frc.robot.commands.MechanismCommands.RobotState;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private final CANBus bus = new CANBus(Constants.CAN_BUS);
  private final Shooter shooter = new Shooter(bus);
  private final Hood hood = new Hood(bus);
  private final Feeder feeder = new Feeder(bus);
  private final MechanismCommands commands = new MechanismCommands(feeder, hood, shooter);

  private final CommandPS4Controller controller =
      new CommandPS4Controller(Constants.CONTROLLER_PORT);
  private final Trigger intakeButton = controller.triangle();
  private final Trigger shootButton = controller.circle();
  private final Trigger ejectButton = controller.cross();
  private final Trigger stopButton = controller.touchpad();

  public RobotContainer() {
    commands.configureBindings(intakeButton, shootButton, ejectButton, stopButton);
  }

  public void stop() {
    commands.changeState(RobotState.IDLE);
  }

}
