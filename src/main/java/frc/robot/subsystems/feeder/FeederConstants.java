package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;

public class FeederConstants {

  public enum FeederState {
    FEEDER,
    REVERSE,
    IDLE
  }

  public static final boolean attached = true;

  public static final int id = -1;

  public static final double p = 1;
  public static final double i = 0;
  public static final double d = 0;

  public static final boolean invert = false;
  public static final boolean breakType = false;
  public static final double gearRatio = 1 / 1;

  public static final Current stallLimit = Units.Amps.of(60);
  public static final Current supplyLimit = Units.Amps.of(80);
  public static final double maxForwardOutput = 0.5;
  public static final double maxReverseOutput = -0.5;

  public static final double FeederSpeed = 0.5;
  public static final double reverseSpeed = -0.5;
  public static final double idleSpeed = 0.0;

  public enum FeederModes {
    FEEDER(FeederSpeed),
    REVERSE(reverseSpeed),
    IDLE(idleSpeed);

    public double output;

    FeederModes(double output) {
      this.output = output;
    }
  }
}
