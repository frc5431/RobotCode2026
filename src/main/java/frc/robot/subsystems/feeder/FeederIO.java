package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean feederConnected = false;
    public double appliedVoltage = 0.0;
    public double RPM = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setFeederVoltage(double voltage) {}

  /** Run the motor at the specified percent output. */
  public default void setPercentOutput(double percent) {}
  
}
