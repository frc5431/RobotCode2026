package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.util.SparkUtil.*;


import frc.team5431.titan.core.subsystem.REVMechanism;

public class FeederIOSparkFlex implements FeederIO {
    private final SparkFlex sparkFlex = new SparkFlex(FeederConstants.id, MotorType.kBrushless);


    public static class FeederSparkFlexConfig extends REVMechanism.Config {
        public FeederSparkFlexConfig() {
        super("PivotSparkFlex", FeederConstants.id);
        configPIDGains(FeederConstants.p, FeederConstants.i, FeederConstants.d);
        configSmartCurrentLimit(FeederConstants.stallLimit, FeederConstants.supplyLimit);
        configSmartStallCurrentLimit(FeederConstants.stallLimit);
        }
    } 

    public FeederIOSparkFlex() {
        
        sparkFlex.configure(new FeederSparkFlexConfig().sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        ifOk(sparkFlex, sparkFlex::getBusVoltage, (value) -> inputs.appliedVoltage = value);
        ifOk(sparkFlex, sparkFlex::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    @Override
    public void setFeederVoltage(double voltage) {
        sparkFlex.setVoltage(voltage);
    }  

}
