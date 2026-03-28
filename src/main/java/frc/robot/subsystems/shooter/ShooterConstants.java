package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.FeedbackSensor;

// import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterConstants {

  public enum ShooterModes {
    SHOOT_FAR(Units.RPM.of(2800), Units.Volts.of(5)),
    SHOOT_CLOSE(Units.RPM.of(2000), Units.Volts.of(
        5)),
    IDLE(Units.RPM.of(0), Units.Volts.of(
        0)),
    REVERSE(Units.RPM.of(-500), Units.Volts.of(-3));

    public AngularVelocity flywheelSpeed;
    public Voltage feederVoltage;

    ShooterModes(AngularVelocity flywheelSpeed, Voltage feederVoltage) {
      this.flywheelSpeed = flywheelSpeed;
      this.feederVoltage = feederVoltage;
    }
  }

  public static class ShooterFlywheelConstants {
    public static final boolean attached = true;

    public static final int leftBackId = 51;
    public static final int leftFrontId = 52;

    public static final int rightFrontId = 54;
    public static final int rightBackId = 53;

    public static final boolean inverted = true;
    public static final boolean breakType = false;
    public static final double gearRatio = 1 / 1;

    public static final double p = 0.002000;
    public static final double i = 0;
    public static final double d = 0;

    public static LoggedNetworkNumber testp = new LoggedNetworkNumber("/Tuning/Shooter/P", 0.002);
    public static final LoggedNetworkNumber testi = new LoggedNetworkNumber("/Tuning/Shooter/I", 0);
    public static final LoggedNetworkNumber testd = new LoggedNetworkNumber("/Tuning/Shooter/D", 0.0);
    public static final LoggedNetworkNumber testkS = new LoggedNetworkNumber("/Tuning/Shooter/kS", 0.33);
    public static final LoggedNetworkNumber testkV = new LoggedNetworkNumber("/Tuning/Shooter/kV", 0.002);
    public static final LoggedNetworkNumber tuneDesiredSpeed = new LoggedNetworkNumber("/Tuning/Shooter/desiredSpeed",0);

    public static final double kS = 0;
    public static final double kV = 0.001000; // feedforward
    // public static final double maxIAccum = 2 * i; //CTRE Doesn't have one? Might
    // Add later

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.RotorSensor;
    public static final FeedbackSensor feedbackSensorREV = FeedbackSensor.kPrimaryEncoder;
    public static final Current stallLimit = Units.Amps.of(60);
    public static final Current supplyLimit = Units.Amps.of(80);

  }

  public class ShooterFeederConstants {
    public static final boolean attached = true;

    public static final int leaderId = 55;
    public static final int followerId = 56;

    public static LoggedNetworkNumber p = new LoggedNetworkNumber("/Tuning/Shooter/Feeder/P", 0.0001);
    public static final LoggedNetworkNumber i = new LoggedNetworkNumber("/Tuning/Shooter/Feeder/I", 0);
    public static final LoggedNetworkNumber d = new LoggedNetworkNumber("/Tuning/Shooter/Feeder/D", 0.0);
    public static final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Shooter/Feeder/kS", 0.1875);
    public static final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Shooter/Feeder/kV", 0.0017);
    public static final LoggedNetworkNumber tuneDesiredSpeed = new LoggedNetworkNumber("/Tuning/Shooter/Feeder/desiredSpeed",0);


    public static final boolean invert = false;
    public static final boolean breakType = false;
    public static final double gearRatio = 1 / 1;

    // stall is current at 0 rpm, supply is current at runnign speed. Stator is
    // current throguh "motor widning" whatever that means.
    public static final Current stallLimit = Units.Amps.of(90);
    public static final Current supplyLimit = Units.Amps.of(80);

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.RotorSensor;
    public static final FeedbackSensor feedbackSensorREV = FeedbackSensor.kPrimaryEncoder;

    public static final LoggedNetworkNumber tuneDesiredFeederVoltage = new LoggedNetworkNumber("/Tuning/Shooter/desiredFeederVoltage",
        0);
  }
}
