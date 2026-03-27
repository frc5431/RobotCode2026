package frc.robot.subsystems.shooter.feeder;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFeederConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class FeederIOTalonFX implements FeederIO {
    private final TalonFX leader = new TalonFX(ShooterFeederConstants.leaderId, Constants.RIO_CANBUS);
    private final TalonFX follower = new TalonFX(ShooterFeederConstants.followerId, Constants.RIO_CANBUS);

    public static class FeederIOTalonFXConfig extends CTREMechanism.Config {
        public FeederIOTalonFXConfig() {
            super("Feeder", Constants.RIO_CANBUS);
            configNeutralBrakeMode(ShooterFeederConstants.breakType);
            configSupplyCurrentLimit(ShooterFeederConstants.supplyLimit);
            configGearRatio(ShooterFeederConstants.gearRatio);
            configMotorInverted(ShooterFeederConstants.invert);
        }
    }

    private StatusSignal<Voltage> leaderAppliedVoltage;
    private StatusSignal<AngularVelocity> leaderRPM;
    private StatusSignal<Current> leaderCurrentAmps;

    private StatusSignal<Voltage> followerAppliedVoltage;
    private StatusSignal<AngularVelocity> followerRPM;
    private StatusSignal<Current> followerCurrentAmps;
    
    private final Debouncer feederConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private FeederIOTalonFXConfig config = new FeederIOTalonFXConfig();

    public final PIDController pid = new PIDController(ShooterFlywheelConstants.testp.get(), ShooterFlywheelConstants.testi.get(), ShooterFlywheelConstants.testd.get());

    public FeederIOTalonFX() {
        leaderAppliedVoltage = leader.getMotorVoltage();
        leaderRPM = leader.getVelocity();
        leaderCurrentAmps = leader.getSupplyCurrent();

        followerAppliedVoltage = follower.getMotorVoltage();
        followerRPM = follower.getVelocity();
        followerCurrentAmps = follower.getSupplyCurrent();

        config.applyTalonConfig(leader);
        config.applyTalonConfig(follower);

        // will need to config whether aligned or inverted later
        follower.setControl(new Follower(ShooterFeederConstants.leaderId, MotorAlignmentValue.Opposed));

        BaseStatusSignal.setUpdateFrequencyForAll(50, leaderAppliedVoltage, leaderCurrentAmps, leaderRPM, followerAppliedVoltage, followerCurrentAmps, followerRPM);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        var feederStatus = BaseStatusSignal.refreshAll(leaderAppliedVoltage, leaderCurrentAmps, leaderRPM, followerAppliedVoltage, followerCurrentAmps, followerRPM);

        inputs.feederConnected = feederConnectedDebounce.calculate(feederStatus.isOK());
        inputs.leaderApliedVoltage = leaderAppliedVoltage.getValueAsDouble();
        inputs.leaderRPM = leaderRPM.getValue().in(RPM);
        inputs.leaderCurrentAmps = leaderCurrentAmps.getValueAsDouble();
        inputs.followerApliedVoltage = followerAppliedVoltage.getValueAsDouble();
        inputs.followerRPM = followerRPM.getValue().in(RPM);
        inputs.followerCurrentAmps = followerCurrentAmps.getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        leader.setVoltage(voltage);
    }

    @Override
    public void setPercentOutput(double percent) {
        leader.set(percent);
    }
}
