package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX leader = new TalonFX(IntakeRollerConstants.leaderId, Constants.RIO_CANBUS);
  private final TalonFX follower = new TalonFX(IntakeRollerConstants.followerId, Constants.RIO_CANBUS);

  public static class RollerTalonFXConfig extends CTREMechanism.Config {
    public RollerTalonFXConfig() {
      super("RollerTalonFX", Constants.RIO_CANBUS);
      configNeutralBrakeMode(IntakeRollerConstants.breakType);
      configFeedbackSensorSource(IntakeRollerConstants.feedbackSensorCTRE);
      configSupplyCurrentLimit(IntakeRollerConstants.supplyLimit);
    }
  }

  private StatusSignal<Voltage> leaderAppliedVoltage;
  private StatusSignal<AngularVelocity> leaderRPM;
  private StatusSignal<Current> leaderCurrentAmps;

  private StatusSignal<Voltage> followerAppliedVoltage;
  private StatusSignal<AngularVelocity> followerRPM;
  private StatusSignal<Current> followerCurrentAmps;

  // No clue what this means copied from ModuleIO
  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private RollerTalonFXConfig config = new RollerTalonFXConfig();

  public RollerIOTalonFX() {
    leaderAppliedVoltage = leader.getMotorVoltage();
    leaderRPM = leader.getVelocity();
    leaderCurrentAmps = leader.getSupplyCurrent();

    followerAppliedVoltage = follower.getMotorVoltage();
    followerRPM = follower.getVelocity();
    followerCurrentAmps = follower.getSupplyCurrent();

    config.applyTalonConfig(leader);
    config.applyTalonConfig(follower);

     // will need to config whether aligned or inverted later
    follower.setControl(new Follower(IntakeRollerConstants.leaderId, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(50, leaderAppliedVoltage, leaderCurrentAmps, leaderRPM, followerAppliedVoltage, followerCurrentAmps, followerRPM);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    var rollerStatus = BaseStatusSignal.refreshAll(leaderAppliedVoltage, leaderCurrentAmps, leaderRPM,
        followerAppliedVoltage, followerCurrentAmps, followerRPM);

    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.leaderAppliedVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderRPM = leaderRPM.getValue().in(RPM);
    inputs.leaderCurrentAmps = leaderCurrentAmps.getValueAsDouble();

    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerRPM = followerRPM.getValue().in(RPM);
    inputs.followerCurrentAmps = followerCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setRollerVoltage(double voltage) {
    leader.setVoltage(voltage);
  }
}
