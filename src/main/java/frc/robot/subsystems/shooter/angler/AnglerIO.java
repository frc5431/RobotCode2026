package frc.robot.subsystems.shooter.angler;

import org.littletonrobotics.junction.AutoLog;

public interface AnglerIO {
  @AutoLog
  public static class AnglerIOInputs {
    public boolean anglerConnected = false;
    public double appliedVoltage = 0.0;
    public double positionAngle = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AnglerIOInputs inputs) {}

  /** Run the motor to the specified rotation per minute. */
  public default void setRPM(double rpm) {}
}
