package frc.robot.subsystems.intake.roller;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;

import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class RollerIOSparkFlex implements RollerIO {
    private final SparkFlex sparkFlex = new SparkFlex(0, null);
    private final RelativeEncoder encoder = sparkFlex.getEncoder();
    public static class RollerIOSparkFlexConfig extends REVMechanism.Config {
        public RollerIOSparkFlexConfig() {
        super("RollerSparkFlex", IntakeRollerConstants.id);
        configPIDGains(IntakeRollerConstants.p, IntakeRollerConstants.i, IntakeRollerConstants.d);
        configFeedbackSensorSource(IntakeRollerConstants.feedbackSensorREV);
        // configGear(RollerIOConstants.gearRatio);
        // configGravity(RollerIOConstants.gravityType);
        configSmartCurrentLimit(IntakeRollerConstants.stallLimit, IntakeRollerConstants.supplyLimit);
        configSmartStallCurrentLimit(IntakeRollerConstants.stallLimit);
        configReverseSoftLimit(
            IntakeRollerConstants.maxReverseRotation, IntakeRollerConstants.useRMaxRotation);
        configForwardSoftLimit(
          IntakeRollerConstants.maxFowardRotation, IntakeRollerConstants.useFMaxRotation);
        }
    } 

    public RollerIOSparkFlex() {
        sparkFlex.configure(new RollerIOSparkFlexConfig().sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        ifOk(sparkFlex, encoder::getVelocity, (value) -> inputs.RPM = value);
        ifOk(sparkFlex, sparkFlex::getBusVoltage, (value) -> inputs.appliedVoltage = value);
        ifOk(sparkFlex, sparkFlex::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    @Override
    public void setRPM(double rpm) {
        sparkFlex
          .getClosedLoopController()
          .setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        sparkFlex.setVoltage(voltage);
    }
}
