package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  public enum FeederState { IDLE, HOLD, SHOOT, FORWARD }

  private final TalonFX feederMotor;
  private final DigitalInput emptySensor = new DigitalInput(Constants.Feeder.SENSOR_DIO);
  private final StatusSignal<Current> currentSignal;
  private FeederState state = FeederState.IDLE;

  public Feeder(CANBus bus) {
    feederMotor = new TalonFX(Constants.Feeder.MOTOR_ID, bus);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = Constants.Feeder.SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.StatorCurrentLimit = Constants.Feeder.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = Constants.Feeder.MOTOR_TO_MECHANISM_RATIO;
    feederMotor.getConfigurator().apply(config);
    currentSignal = feederMotor.getStatorCurrent();
    currentSignal.setUpdateFrequency(50);
  }

  public void setState(FeederState state) {
    this.state = state;
  }

  public FeederState getState() {
    return state;
  }

  public void setMotorVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  public boolean hasBalls() {
    return emptySensor.get() != Constants.Feeder.SENSOR_TRUE_WHEN_EMPTY;
  }

  public double getCurrent() {
    return currentSignal.getValueAsDouble();
  }

  public void runFeeder() {
    switch (state) {
      case IDLE:
      case HOLD:
        setMotorVoltage(0);
        break;
      case SHOOT:
      case FORWARD:
        if (hasBalls()) {
          setMotorVoltage(Constants.Feeder.FORWARD_VOLTS);
        } else {
          setMotorVoltage(0);
        }
        break;
    }
  }

  @Override
  public void periodic() {
    currentSignal.refresh();
    if (DriverStation.isDisabled()) {
      state = FeederState.IDLE;
    }
    runFeeder();
  }
}
