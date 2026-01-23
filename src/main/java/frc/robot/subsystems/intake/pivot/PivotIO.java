package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public boolean pivotConnected = false;
    public double appliedVoltage = 0.0;
    public double position = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setPivotVoltage(double voltage) {}

  /** Run the motor to the specified position. */
  public default void setPosition(double positionAngle) {}
}