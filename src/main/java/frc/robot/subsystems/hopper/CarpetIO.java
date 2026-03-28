package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface CarpetIO {
  @AutoLog
  public static class CarpetIOInputs {
    public boolean rollerConnected = false;
    public double appliedVoltage = 0.0;
    public double RPM = 0.0;
    public double currentAmps = 0.0;
    public double setpointRPM = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CarpetIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setRollerVoltage(double voltage) {}

  public default void setRPM(AngularVelocity rpm) {}

}
