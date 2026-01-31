package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {

  // public enum ShooterModes {
  //   SHOOT( Units.RPM.of(2000)),
  //   IDLE( Units.RPM.of(0));

  //   public AngularVelocity speed;

  //   ShooterModes(AngularVelocity speed) {
  //     this.speed = speed;
  //   }
  // }

  public static class ShooterFlywheelConstants {
    public static final boolean attached = true;
    public static final int followerId = 50;
    public static final int leaderId = 50;
    public static final boolean inverted = false;
    public static final boolean breakType = false;
    public static final double gearRatio = 1 / 1;

    public static final double p = 1;
    public static final double i = 0;
    public static final double d = 0;
    // public static final double maxIAccum = 2 * i; //CTRE Doesn't have one? Might
    // Add later

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.FusedCANcoder;

    public static final Current stallLimit = Units.Amps.of(60);
    public static final Current supplyLimit = Units.Amps.of(80);

  }
  
  public static class ShooterAnglerConstants {
    public static final boolean attached = true;
    public static final int id = 50;
    public static final boolean inverted = false;
    public static final boolean breakType = false;
    public static final double gearRatio = 1 / 1;

    public static final double p = 1;
    public static final double i = 0;
    public static final double d = 0;
    // public static final double maxIAccum = 2 * i; //CTRE Doesn't have one? Might
    // Add later

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.FusedCANcoder;
    
    public static final Current stallLimit = Units.Amps.of(60);
    public static final Current supplyLimit = Units.Amps.of(80);
    public static final double maxForwardOutput = 1;
    public static final double maxReverseOutput = 0.5;

  }
}
