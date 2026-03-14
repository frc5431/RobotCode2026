package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;

import static frc.robot.util.SparkUtil.*;

public class FeederIOSparkFlex implements FeederIO {
    private final SparkFlex sparkFlex = new SparkFlex(FeederConstants.id, MotorType.kBrushless);
    private final SparkFlexConfig config = new SparkFlexConfig();

    public FeederIOSparkFlex() {
        // CONFIG
        config.closedLoop.feedbackSensor(FeederConstants.feedbackSensorREV);
        config.smartCurrentLimit(
          (int) FeederConstants.stallLimit.in(Units.Amps), (int) FeederConstants.supplyLimit.in(Units.Amps));
        config.closedLoop.pid(FeederConstants.p, FeederConstants.i, FeederConstants.d, ClosedLoopSlot.kSlot0);
        
        
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
