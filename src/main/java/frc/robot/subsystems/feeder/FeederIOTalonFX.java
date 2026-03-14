package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class FeederIOTalonFX implements FeederIO {
    private final TalonFX talon = new TalonFX(FeederConstants.id, Constants.RIO_CANBUS);

    public static class FeederIOTalonFXConfig extends CTREMechanism.Config {
        public FeederIOTalonFXConfig() {
            super("Feeder", Constants.RIO_CANBUS);

            configNeutralBrakeMode(FeederConstants.breakType);
            configStatorCurrentLimit(FeederConstants.stallLimit);
            configSupplyCurrentLimit(FeederConstants.supplyLimit);
            configForwardSoftLimit(FeederConstants.maxForwardOutput, true);
            configReverseSoftLimit(FeederConstants.maxReverseOutput, true);
            configPIDGains(FeederConstants.p, FeederConstants.i, FeederConstants.d);
            configPeakOutput(FeederConstants.maxForwardOutput, FeederConstants.maxReverseOutput);
            configGearRatio(FeederConstants.gearRatio);
            configMotorInverted(FeederConstants.invert);
        }
    }

    private StatusSignal<Voltage> appliedVoltage;
    private StatusSignal<AngularVelocity> feederRPM;
    private StatusSignal<Current> currentAmps;

    private final Debouncer feederConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private FeederIOTalonFXConfig config = new FeederIOTalonFXConfig();

    public FeederIOTalonFX() {
        appliedVoltage = talon.getMotorVoltage();
        feederRPM = talon.getVelocity();
        currentAmps = talon.getSupplyCurrent();
        config.applyTalonConfig(talon);

        BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, currentAmps, feederRPM);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        var feederStatus = BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, feederRPM);

        inputs.feederConnected = feederConnectedDebounce.calculate(feederStatus.isOK());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.RPM = feederRPM.getValue().in(RPM);
        inputs.currentAmps = currentAmps.getValueAsDouble();
    }

    @Override
    public void setFeederVoltage(double voltage) {
        talon.setVoltage(voltage);
    }

    @Override
    public void setPercentOutput(double percent) {
        talon.set(percent);
    }
}
