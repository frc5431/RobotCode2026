// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  

//TUNING
public static final LoggedNetworkBoolean TuningMode =
        new LoggedNetworkBoolean("/Tuning/TuningMode", false);

//SHOOTER
public static final LoggedNetworkBoolean ShooterEnabled =
        new LoggedNetworkBoolean("/Tuning/Shooter/ShooterEnabled", false);

public static final LoggedNetworkNumber TuningMode_Shooter_RPM =
        new LoggedNetworkNumber("/Tuning/Shooter/Shooter_RPM", 3000);

public static final LoggedNetworkNumber Feeder_RPM =
        new LoggedNetworkNumber("/Tuning/Shooter/Feeder_RPM", 2850);

//CARPET
public static final LoggedNetworkBoolean CarpetEnabled =
        new LoggedNetworkBoolean("/Tuning/Carpet/CarpetEnabled", false);

public static final LoggedNetworkNumber Carpet_RPM =
        new LoggedNetworkNumber("/Tuning/Carpet/Carpet_RPM", 6000);

//INTAKE
public static final LoggedNetworkBoolean IntakeEnabled =
        new LoggedNetworkBoolean("/Tuning/Intake/IntakeEnabled", false);

public static final LoggedNetworkNumber Intake_RPM =
        new LoggedNetworkNumber("/Tuning/Intake/Intake_RPM", 6000);

//PIVOT PULSE

public static final LoggedNetworkBoolean PIVOT_PULSE =
        new LoggedNetworkBoolean("/Tuning/Intake/PivotPulse", false);

//AIMING
public static final LoggedNetworkNumber Align_Tolerance_Deg =
        new LoggedNetworkNumber("/Tuning/Aiming/AlignToleranceDeg", 3.0);
// DRIVE SPEED
public static final LoggedNetworkNumber DriveSpeedMultipler =
        new LoggedNetworkNumber("/Tuning/Drive/DriveSpeedMultiplier", 0.8);


  // CAN bus that the devices are located on;
  public static final CANBus CANIVORE_CANBUS = new CANBus("Canivore", "./logs/example.hoot");
   public static final CANBus RIO_CANBUS = new CANBus();

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


}