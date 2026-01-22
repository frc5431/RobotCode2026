package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
    private DCMotorSim rollerMotorSim;
    private PIDController rollerController;
    private boolean rollerClosedLoop = false;
    private double appliedVoltage = 0.0;
    private double driveFFVolts = 0.0;

    // From ModuleIOSim no clue tbh
    private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private static final double DRIVE_KS = 0.0;


    public RollerIOSim() {
        this.rollerMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), .0004, 1.0), DCMotor.getKrakenX60(1));
        
        // Enable wrapping for turn PID --- TODO: NOT SURE WHAT THIS IS ASK
        rollerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        if (rollerClosedLoop) {
            appliedVoltage = driveFFVolts + rollerController.calculate(rollerMotorSim.getAngularVelocityRadPerSec());
        } else {
            rollerController.reset();
        }

        rollerMotorSim.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
        rollerMotorSim.update(0.02);

        inputs.rollerConnected = true;
        inputs.RPM = rollerMotorSim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
    }

    @Override
    public void setRPM(double RPM) {
        rollerClosedLoop = true;
        driveFFVolts = DRIVE_KS * Math.signum(RPM) + DRIVE_KV * RPM;
        rollerController.setSetpoint(RPM);
    }
    
    public void setAppliedVoltage(double appliedVoltage) {
        this.appliedVoltage = MathUtil.clamp(appliedVoltage, -12.0, 12.0);
    }
}
