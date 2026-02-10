package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean flywheelConnected = false;
    public double leaderAppliedVoltage = 0.0;
    public double leaderRPM = 0.0;
    public double leaderAmps = 0.0;

    public double followerAppliedVoltage = 0.0;
    public double followerRPM = 0.0;
    public double followerAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run the motor to the specified rotation per minute. */
  public default void setRPM(double rpm) {}
}
