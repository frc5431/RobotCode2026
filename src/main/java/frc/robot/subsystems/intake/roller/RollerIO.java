package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  @AutoLog
  public static class RollerIOInputs {
    public boolean driveConnected = false;
    public double appliedVoltage = 0.0;
    public double output = 0.0;
    public double rpm = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(RollerIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setRollerVoltage(double voltage) {}

  /** Run the motor to the specified rotation. */
  public default void setOutput(double output) {}

  public default void setRPM(double rpm) {}
}
