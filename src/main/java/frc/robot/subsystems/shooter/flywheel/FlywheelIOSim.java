package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
    private FlywheelSim flywheelMotorSim;
    private PIDController flywheelController = new PIDController(FLYWHEEL_KP, 0, FLYWHEEL_KD);
    private boolean flywheelClosedLoop = false;
    private double appliedVoltage = 0.0;
    private double flywheelFFVolts = 0.0;
    private double flywheelSetpoint = 0.0;

    // From ModuleIOSim no clue tbh
    private static final double FLYWHEEL_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double FLYWHEEL_KV = FLYWHEEL_KV_ROT / (2.0 * Math.PI);
    private static final double FLYWHEEL_KS = 0.0;
    private static final double FLYWHEEL_KP = .03;
    private static final double FLYWHEEL_KD = 0.0;


    public FlywheelIOSim() {
      LinearSystem<N1, N1,N1> plant = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 1.0, 0.004);
      
        this.flywheelMotorSim = new FlywheelSim( 
            plant, DCMotor.getKrakenX60(2)
                );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        if (flywheelClosedLoop) {
            appliedVoltage = flywheelFFVolts + flywheelController.calculate(flywheelMotorSim.getAngularVelocityRadPerSec());
        } else {
            flywheelController.reset();
        }

        flywheelMotorSim.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
        flywheelMotorSim.update(0.02);

        inputs.flywheelConnected = true;
        
        inputs.leaderRPM = flywheelMotorSim.getAngularVelocityRPM();
        inputs.leaderAppliedVoltage = appliedVoltage;
        inputs.leaderAmps = Math.abs(flywheelMotorSim.getCurrentDrawAmps());

        inputs.followerRPM = flywheelMotorSim.getAngularVelocityRPM();
        inputs.followerAppliedVoltage = appliedVoltage;
        inputs.followerAmps = Math.abs(flywheelMotorSim.getCurrentDrawAmps());
        inputs.setpointRPM = flywheelSetpoint;
    }

    @Override
    public void setRPM(AngularVelocity rpm) {
      flywheelSetpoint = rpm.magnitude();

      if (rpm.in(Units.RPM) == 0) {
        flywheelClosedLoop = false;
        appliedVoltage = 0.0;
        return;
      }

      flywheelClosedLoop = true;

      double setpointRadPerSec = rpm.in(Units.RadiansPerSecond);

      flywheelFFVolts = FLYWHEEL_KS * Math.signum(setpointRadPerSec)
          + FLYWHEEL_KV * setpointRadPerSec;

      flywheelController.setSetpoint(setpointRadPerSec);
    }
}
