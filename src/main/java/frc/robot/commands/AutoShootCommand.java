// package frc.robot.commands;

// import java.util.function.Supplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.hopper.Carpet;
// import frc.robot.subsystems.hopper.CarpetConstants.CarpetModes;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.shooter.ShooterMath;

// public class AutoShootCommand extends Command {

//     private final Intake intake;
//     private final Carpet carpet;
//     private final Shooter shooter;
//     private final Supplier<Double> distToHub;
//     private double cyclesAtTarget;
//     private boolean speedAchieved;

//     public AutoShootCommand(Intake intake, Carpet carpet, Shooter shooter, Drive drive) {
//     // addCommands(
//     //   // new ParallelRaceGroup(
//     //   //   feeder.runFeederCommand(FeederModes.REVERSE),
//     //   //   new WaitCommand(0.5)),
//     //   new ParallelCommandGroup(
//     //     new InhaleCommand(intake, carpet, feeder,true, true).withName("ShootFuelCommand.Inhale")),
//     //     shooter.runShooterCommand(ShooterModes.SHOOT_CLOSE).withName("ShootFuelCommand.Shoot")
//     //   );
      
//     addRequirements(intake, carpet, shooter);

//     this.intake = intake;
//     this.carpet = carpet;
//     this.shooter = shooter;
//     distToHub = drive::distFromHub;
//   }

//     @Override
//     public void execute() {
//         double desiredRPM = ShooterMath.calculateSpeed(distToHub.get());
//         double desiredPos = ShooterMath.calculateHoodPosition(distToHub.get());

//         shooter.runShooterCustom(desiredRPM, desiredPos);

//         if (!MathUtil.isNear(desiredPos, shooter.getPosition(), ShooterAnglerConstants.tolerance)) {
//             cyclesAtTarget = 0;
//         } else {
//             cyclesAtTarget++;
//         }

//         if (MathUtil.isNear(desiredRPM, shooter.getFlywheelSpeed(), desiredRPM * 0.05)) {
//             speedAchieved = true;
//         }

//         if (cyclesAtTarget > 5 && speedAchieved) {
//             intake.runIntakeEnum(IntakeMode.INTAKE);
//             carpet.runRollerEnum(CarpetModes.INTAKE);
//             feeder.runFeederEnum(FeederModes.FEEDER);
//         }
//     }
// }
