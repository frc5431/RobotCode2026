package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  @AutoLog
  public static class RollerIOInputs {
    public boolean rollerConnected = false;
    public double appliedVoltage = 0.0;
    public double RPM = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(RollerIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setRollerVoltage(double voltage) {}

  /** Run the motor to the specified rotation per minute. */
  public default void setRPM(double rpm) {}
}
