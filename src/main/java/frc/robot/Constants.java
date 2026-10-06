package frc.robot;

public final class Constants {
  private Constants() {}

  public static final String CAN_BUS = "rio";
  public static final int CONTROLLER_PORT = 0;

  public static final class Feeder {
    public static final int MOTOR_ID = 1;
    public static final int SENSOR_DIO = 0;
    public static final boolean SENSOR_TRUE_WHEN_EMPTY = true;
    public static final double MOTOR_TO_MECHANISM_RATIO = 5.0 / 3.0;
    public static final double FORWARD_VOLTS = 4;
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 60;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 30;
  }

  public static final class Shooter {
    public static final int LEFT_ID = 2;
    public static final int RIGHT_ID = 3;
    public static final double MOTOR_TO_MECHANISM_RATIO = 1.0 / 2.0;
    public static final double SHOOT_RPS = 50;
    public static final double SPEED_TOLERANCE_RPS = 1;
    public static final double EJECT_RPS = 15;
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 80;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 60;
    public static final double PREPARE_TIMEOUT_SECONDS = 3;
    public static final double KP = 0.1;
    public static final double KS = 0.1;
    public static final double KV = 0.06;
  }

  public static final class Hood {
    public static final int MOTOR_ID = 4;
    public static final int CANCODER_ID = 5;
    public static final double MOTOR_TO_MECHANISM_RATIO = 16;
    public static final double SENSOR_TO_MECHANISM_RATIO = 1;
    public static final double MAGNET_OFFSET = 0;
    public static final double SHOOT_DEGREES = 30;
    public static final double EJECT_DEGREES = 10;
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 30;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 15;
    public static final double KP = 20;
  }
}
