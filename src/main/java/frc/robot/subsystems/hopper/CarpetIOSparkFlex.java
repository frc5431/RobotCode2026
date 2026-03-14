package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;

import static frc.robot.util.SparkUtil.*;

import frc.robot.subsystems.hopper.CarpetConstants.CarpetRollerConstants;

public class CarpetIOSparkFlex implements CarpetIO {
    // Neo Vortex
    private final SparkFlex sparkFlex = new SparkFlex(CarpetRollerConstants.id, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder encoder = sparkFlex.getEncoder();
    private final SparkFlexConfig config = new SparkFlexConfig();

    public CarpetIOSparkFlex() {
        // CONFIG
        config.closedLoop.feedbackSensor(CarpetRollerConstants.feedbackSensorREV);
        config.smartCurrentLimit(
          (int) CarpetRollerConstants.stallLimit.in(Units.Amps), (int) CarpetRollerConstants.supplyLimit.in(Units.Amps));
        config.closedLoop.pid(CarpetRollerConstants.p, CarpetRollerConstants.i, CarpetRollerConstants.d, ClosedLoopSlot.kSlot0);
        
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
