package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public enum ShooterState {
    shooting,
    Idle,
    removeBalls,
    prepareToShoot
  }

  private final TalonFX mainMotor;
  private final TalonFX secondMotor;
  private final StatusSignal<AngularVelocity> mainVelocity;
  private final StatusSignal<AngularVelocity> secondVelocity;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage secondVelocityRequest = new VelocityVoltage(0);
  private ShooterState state = ShooterState.Idle;

  public Shooter(CANBus bus) {
    mainMotor = new TalonFX(Constants.Shooter.LEFT_ID, bus);
    secondMotor = new TalonFX(Constants.Shooter.RIGHT_ID, bus);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.StatorCurrentLimit = Constants.Shooter.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = Constants.Shooter.MOTOR_TO_MECHANISM_RATIO;
    config.Slot0.kP = Constants.Shooter.KP;
    config.Slot0.kS = Constants.Shooter.KS;
    config.Slot0.kV = Constants.Shooter.KV;
    mainMotor.getConfigurator().apply(config);
    secondMotor.getConfigurator().apply(config);
    mainVelocity = mainMotor.getVelocity();
    secondVelocity = secondMotor.getVelocity();
    mainVelocity.setUpdateFrequency(50);
    secondVelocity.setUpdateFrequency(50);
  }

  public void setState(ShooterState state) {
    this.state = state;
  }

  public ShooterState getState() {
    return state;
  }

  public void setMotorVelocity(double velocity) {
    mainMotor.setControl(velocityRequest.withVelocity(velocity));
    secondMotor.setControl(secondVelocityRequest.withVelocity(-velocity));
  }

  public void runShooter() {
    switch (state) {
      case Idle:
        mainMotor.setVoltage(0);
        secondMotor.setVoltage(0);
        break;
      case shooting:
        setMotorVelocity(Constants.Shooter.SHOOT_RPS);
        break;
      case removeBalls:
        setMotorVelocity(-Constants.Shooter.EJECT_RPS);
        break;
      case prepareToShoot:
        setMotorVelocity(Constants.Shooter.SHOOT_RPS);
        break;
    }
  }

  public double getVelocity() {
    return mainVelocity.getValueAsDouble();
  }

  public boolean isAtSpeed() {
    double target = Constants.Shooter.SHOOT_RPS;
    double tolerance = Constants.Shooter.SPEED_TOLERANCE_RPS;
    boolean mainReady = Math.abs(mainVelocity.getValueAsDouble() - target) <= tolerance;
    boolean secondReady = Math.abs(secondVelocity.getValueAsDouble() + target) <= tolerance;
    return mainVelocity.getStatus().isOK() && secondVelocity.getStatus().isOK()
        && mainReady && secondReady;
  }

  @Override
  public void periodic() {
    mainVelocity.refresh();
    secondVelocity.refresh();
    if (DriverStation.isDisabled()) {
      state = ShooterState.Idle;
    }
    runShooter();
  }
}