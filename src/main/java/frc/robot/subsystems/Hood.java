package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  public enum HoodState { IDLE, SHOOT, EJECT }

  private final TalonFX hoodMotor;
  private final CANcoder angleEncoder;
  private final StatusSignal<Angle> angleSignal;
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private HoodState state = HoodState.IDLE;

  public Hood(CANBus bus) {
    hoodMotor = new TalonFX(Constants.Hood.MOTOR_ID, bus);
    angleEncoder = new CANcoder(Constants.Hood.CANCODER_ID, bus);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Hood.MAGNET_OFFSET;
    angleEncoder.getConfigurator().apply(encoderConfig);
    // RemoteCANcoder control consumes the CANcoder's position publications.
    angleEncoder.getPosition().setUpdateFrequency(50);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.StatorCurrentLimit = Constants.Hood.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.SensorToMechanismRatio = Constants.Hood.SENSOR_TO_MECHANISM_RATIO;
    config.Feedback.RotorToSensorRatio = Constants.Hood.MOTOR_TO_MECHANISM_RATIO
        / Constants.Hood.SENSOR_TO_MECHANISM_RATIO;
    config.Slot0.kP = Constants.Hood.KP;
    hoodMotor.getConfigurator().apply(config);
    angleSignal = hoodMotor.getPosition();
    angleSignal.setUpdateFrequency(50);
  }

  public void setState(HoodState state) {
    this.state = state;
  }

  public HoodState getState() {
    return state;
  }

  public void setMotorPosition(double degrees) {
    hoodMotor.setControl(positionRequest.withPosition(degrees / 360));
  }

  public void runHood() {
    switch (state) {
      case IDLE:
        hoodMotor.setVoltage(0);
        break;
      case SHOOT:
        setMotorPosition(Constants.Hood.SHOOT_DEGREES);
        break;
      case EJECT:
        setMotorPosition(Constants.Hood.EJECT_DEGREES);
        break;
    }
  }

  public double getAngle() {
    return angleSignal.getValueAsDouble() * 360;
  }

  @Override
  public void periodic() {
    angleSignal.refresh();
    if (DriverStation.isDisabled()) {
      state = HoodState.IDLE;
    }
    runHood();
  }
}
