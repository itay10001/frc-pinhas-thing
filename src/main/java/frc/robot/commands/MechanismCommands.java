package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class MechanismCommands {
  public enum RobotState { IDLE, INTAKE, HOLD, SHOOTING, EJECT, OPEN_WALLS }

  private final Feeder feeder;
  private final Hood hood;
  private final Shooter shooter;
  private RobotState state = RobotState.IDLE;

  private boolean magazineFull = false;
  private boolean inAllianceZone = true;
  private boolean activePeriod = true;
  private boolean intakeClosed = false;
  private boolean intakeOpen = true;
  private boolean intakeHasOpened = true;

  public MechanismCommands(Feeder feeder, Hood hood, Shooter shooter) {
    this.feeder = feeder;
    this.hood = hood;
    this.shooter = shooter;
  }

  public RobotState getState() {
    return state;
  }

  public void configureBindings(Trigger intakeButton, Trigger shootButton,
      Trigger ejectButton, Trigger stopButton) {
    Trigger enabled = new Trigger(DriverStation::isEnabled);

    Trigger idle = new Trigger(() -> stopButton.getAsBoolean()
        || (state == RobotState.INTAKE && !intakeButton.getAsBoolean() && !feeder.hasBalls())
        || ((state == RobotState.EJECT || ejectButton.getAsBoolean()) && !feeder.hasBalls())
        || (state == RobotState.OPEN_WALLS && intakeOpen)
        || ((state == RobotState.SHOOTING || shootButton.getAsBoolean()) && !feeder.hasBalls()));

    Trigger eject = ejectButton.and(feeder::hasBalls).and(idle.negate());
    Trigger shoot = shootButton.and(enabled)
        .and(() -> feeder.hasBalls() && inAllianceZone && activePeriod)
        .and(eject.negate()).and(idle.negate());
    Trigger intake = intakeButton.and(() -> !magazineFull)
        .and(shoot.negate()).and(eject.negate()).and(idle.negate());
    Trigger hold = new Trigger(() -> state == RobotState.INTAKE && feeder.hasBalls()
        && (!intakeButton.getAsBoolean() || magazineFull))
        .and(shoot.negate()).and(eject.negate()).and(idle.negate());
    Trigger openWalls = new Trigger(() -> intakeClosed && !intakeHasOpened)
        .and(idle.negate()).and(eject.negate()).and(shoot.negate())
        .and(intake.negate()).and(hold.negate());

    openWalls.and(enabled).onTrue(createStateCommand(RobotState.OPEN_WALLS));
    intake.and(enabled).onTrue(createStateCommand(RobotState.INTAKE));
    shoot.and(enabled).onTrue(createStateCommand(RobotState.SHOOTING));
    eject.and(enabled).onTrue(createStateCommand(RobotState.EJECT));
    hold.and(enabled).onTrue(createStateCommand(RobotState.HOLD));
    idle.and(enabled).onTrue(createStateCommand(RobotState.IDLE));
  }

  public Command createStateCommand(RobotState newState) {
    if (newState == RobotState.SHOOTING) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> changeState(newState), feeder, hood, shooter),
          new WaitUntilCommand(shooter::isAtSpeed)
              .withTimeout(Constants.Shooter.PREPARE_TIMEOUT_SECONDS),
          new InstantCommand(() -> {
            if (state == RobotState.SHOOTING && feeder.hasBalls() && shooter.isAtSpeed()) {
              shooter.setState(Shooter.ShooterState.shooting);
              feeder.setState(Feeder.FeederState.SHOOT);
            } else if (state == RobotState.SHOOTING) {
              changeState(RobotState.IDLE);
            }
          }, feeder, shooter))
          .finallyDo(interrupted -> {
            if (interrupted && state == RobotState.SHOOTING) {
              changeState(RobotState.IDLE);
            }
          });
    }
    return new InstantCommand(() -> changeState(newState), feeder, hood, shooter);
  }

  public void changeState(RobotState newState) {
    state = newState;
    switch (state) {
      case SHOOTING:
        feeder.setState(Feeder.FeederState.IDLE);
        hood.setState(Hood.HoodState.SHOOT);
        shooter.setState(Shooter.ShooterState.prepareToShoot);
        break;
      case EJECT:
        feeder.setState(Feeder.FeederState.FORWARD);
        hood.setState(Hood.HoodState.EJECT);
        shooter.setState(Shooter.ShooterState.removeBalls);
        break;
      case HOLD:
        feeder.setState(Feeder.FeederState.HOLD);
        hood.setState(Hood.HoodState.IDLE);
        shooter.setState(Shooter.ShooterState.Idle);
        break;

      case IDLE:
      case INTAKE:
      case OPEN_WALLS:
        feeder.setState(Feeder.FeederState.IDLE);
        hood.setState(Hood.HoodState.IDLE);
        shooter.setState(Shooter.ShooterState.Idle);
        break;
    }
  }
}
