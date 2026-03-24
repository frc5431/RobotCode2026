package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean flywheelConnected = false;

    public double leftTopLeaderAppliedVoltage = 0.0;
    public double leftTopLeaderRPM = 0.0;
    public double leftTopLeaderAmps = 0.0;

    public double leftBottomFollowerAppliedVoltage = 0.0;
    public double leftBottomFollowerRPM = 0.0;
    public double leftBottomFollowerAmps = 0.0;

    public double rightTopFollowerAppliedVoltage = 0.0;
    public double rightTopFollowerRPM = 0.0;
    public double rightTopFollowerAmps = 0.0;

    public double rightBottomFollowerAppliedVoltage = 0.0;
    public double rightBottomFollowerRPM = 0.0;
    public double rightBottomFollowerAmps = 0.0;

    public double setpointRPM = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run the motor to the specified rotation per minute. */
  public default void setRPM(AngularVelocity rpm) {}

  public default void setVoltage(double voltage) {}
}
