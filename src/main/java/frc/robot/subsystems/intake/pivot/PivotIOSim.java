package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
    private DCMotorSim pivotMotorSim;
    private PIDController pivotContpivot = new PIDController(PIVOT_KP, 0, PIVOT_KD);
    private boolean pivotClosedLoop = false;
    private double appliedVoltage = 0.0;
    private double pivotFFVolts = 0.0;

    // From ModuleIOSim no clue tbh
    private static final double pivot_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double PIVOT_KV = 1.0 / Units.rotationsToRadians(1.0 / pivot_KV_ROT);
    private static final double PIVOT_KS = 0.0;
    private static final double PIVOT_KP = 1.0;
    private static final double PIVOT_KD = 0.0;


    public PivotIOSim() {
        this.pivotMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), .0004, 1.0), DCMotor.getKrakenX60(1));
        
        // Enable wrapping for turn PID --- TODO: NOT SURE WHAT THIS IS ASK
        pivotContpivot.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        if (pivotClosedLoop) {
            appliedVoltage = pivotFFVolts + pivotContpivot.calculate(pivotMotorSim.getAngularVelocityRadPerSec());
        } else {
            pivotContpivot.reset();
        }

        pivotMotorSim.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
        pivotMotorSim.update(0.02);

        inputs.pivotConnected = true;
        inputs.position = pivotMotorSim.getAngularPositionRad();
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmps = Math.abs(pivotMotorSim.getCurrentDrawAmps());
    }

    @Override
    // TODO
    public void setPosition(double positionAngle) {
      
    }
    
    public void setAppliedVoltage(double appliedVoltage) {
        this.appliedVoltage = MathUtil.clamp(appliedVoltage, -12.0, 12.0);
    }
}
