package frc.robot.subsystems.shooter.feeder;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean feederConnected = false;

    public double leaderApliedVoltage = 0.0;
    public double leaderRPM = 0.0;
    public double leaderCurrentAmps = 0.0;

    public double followerApliedVoltage = 0.0;
    public double followerRPM = 0.0;
    public double followerCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setVoltage(double voltage) {}

  /** Run the motor at the specified percent output. */
  public default void setPercentOutput(double percent) {}
  
  /** Run the motor to the specified rotation per minute. */
  public default void setRPM(AngularVelocity rpm) {}

}
