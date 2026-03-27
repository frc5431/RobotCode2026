package frc.robot.subsystems.shooter.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFeederConstants;

import static frc.robot.util.SparkUtil.*;

public class FeederIOSparkFlex implements FeederIO {
    private final SparkFlex sparkFlex = new SparkFlex(ShooterFeederConstants.leaderId, MotorType.kBrushless);
    private final SparkFlexConfig config = new SparkFlexConfig();

    public FeederIOSparkFlex() {
        // CONFIG
        config.closedLoop.feedbackSensor(ShooterFeederConstants.feedbackSensorREV);
        config.smartCurrentLimit(
          (int) ShooterFeederConstants.stallLimit.in(Units.Amps), (int) ShooterFeederConstants.supplyLimit.in(Units.Amps));
        config.closedLoop.pid(ShooterFeederConstants.p.get(), ShooterFeederConstants.i.get(), ShooterFeederConstants.d.get(), ClosedLoopSlot.kSlot0);
        
        
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        ifOk(sparkFlex, sparkFlex::getBusVoltage, (value) -> inputs.leaderApliedVoltage = value);
        ifOk(sparkFlex, sparkFlex::getOutputCurrent, (value) -> inputs.leaderCurrentAmps = value);
    }

    @Override
    public void setVoltage(double voltage) {
        sparkFlex.setVoltage(voltage);
    }  

}
