package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
  private DCMotorSim flywheelMotorSim;
    private PIDController flywheelController = new PIDController(ROLLER_KP, 0, ROLLER_KD);
    private boolean flywheelClosedLoop = false;
    private double appliedVoltage = 0.0;
    private double rollerFFVolts = 0.0;

    // From ModuleIOSim no clue tbh
    private static final double ROLLER_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double ROLLER_KV = 1.0 / Units.rotationsToRadians(1.0 / ROLLER_KV_ROT);
    private static final double ROLLER_KS = 0.0;
    private static final double ROLLER_KP = 1.0;
    private static final double ROLLER_KD = 0.0;


    public FlywheelIOSim() {
        this.flywheelMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), .0004, 1.0), DCMotor.getKrakenX60(1));
        
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        if (flywheelClosedLoop) {
            appliedVoltage = rollerFFVolts + flywheelController.calculate(flywheelMotorSim.getAngularVelocityRadPerSec());
        } else {
            flywheelController.reset();
        }

        flywheelMotorSim.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
        flywheelMotorSim.update(0.02);

        inputs.flywheelConnected = true;
        inputs.RPM = flywheelMotorSim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.currentAmps = Math.abs(flywheelMotorSim.getCurrentDrawAmps());
    }

    @Override
    public void setRPM(double RPM) {
        flywheelClosedLoop = true;
        rollerFFVolts = ROLLER_KS * Math.signum(RPM) + ROLLER_KV * RPM;
        flywheelController.setSetpoint(RPM);
    }
}
