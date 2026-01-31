package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean flywheelConnected = false;
    public double appliedVoltage = 0.0;
    public double RPM = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run the motor to the specified rotation per minute. */
  public default void setRPM(double rpm) {}
}
