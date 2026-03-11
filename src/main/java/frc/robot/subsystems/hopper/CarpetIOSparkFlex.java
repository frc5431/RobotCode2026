package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;

import static frc.robot.util.SparkUtil.*;

import frc.robot.subsystems.hopper.CarpetConstants.CarpetRollerConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class CarpetIOSparkFlex implements CarpetIO {
    // Neo Vortex
    private final SparkFlex sparkFlex = new SparkFlex(CarpetRollerConstants.id, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder encoder = sparkFlex.getEncoder();

    public static class CarpetIOSparkFlexConfig extends REVMechanism.Config {
        public CarpetIOSparkFlexConfig() {
        super("CarpetSparkFlex", CarpetRollerConstants.id);
        configPIDGains(CarpetRollerConstants.p, CarpetRollerConstants.i, CarpetRollerConstants.d);
        configFeedbackSensorSource(CarpetRollerConstants.feedbackSensorREV);
        // configGear(CarpetIOConstants.gearRatio);
        // configGravity(CarpetIOConstants.gravityType);
        configSmartCurrentLimit(CarpetRollerConstants.stallLimit, CarpetRollerConstants.supplyLimit);
        configSmartStallCurrentLimit(CarpetRollerConstants.stallLimit);
        }
    } 

    public CarpetIOSparkFlex() {
        sparkFlex.configure(new CarpetIOSparkFlexConfig().sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(CarpetIOInputs inputs) {
        ifOk(sparkFlex, encoder::getVelocity, (value) -> inputs.RPM = value);
        ifOk(sparkFlex, sparkFlex::getBusVoltage, (value) -> inputs.appliedVoltage = value);
        ifOk(sparkFlex, sparkFlex::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        sparkFlex.setVoltage(voltage);
    }
}
