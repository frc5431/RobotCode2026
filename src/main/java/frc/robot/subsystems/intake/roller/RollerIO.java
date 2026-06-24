package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface RollerIO {

  @AutoLog
  public static class RollerIOInputs {
    public boolean rollerConnected = false;

    public double leaderAppliedVoltage = 0.0;
    public double leaderRPM = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double setpointRPM = 0.0;

    public double followerAppliedVoltage = 0.0;
    public double followerRPM = 0.0;
    public double followerCurrentAmps = 0.0;

  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(RollerIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setRollerVoltage(double voltage) {}

  /** Run the motor to the specified rotation per minute. */
  public default void setRPM(AngularVelocity rpm) {}
}
