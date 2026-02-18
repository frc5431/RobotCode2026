package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederConstants.FeederModes;
import frc.robot.subsystems.feeder.FeederConstants.FeederState;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;
import lombok.Getter;
import lombok.Setter;

public class Feeder extends SubsystemBase {

    private final FeederIO feederIO;
    private final FeederIOInputs feederInputs = new FeederIOInputs();

    @Getter @Setter private FeederModes feederMode;
    @Getter @Setter private FeederState feederState;

    public Feeder(FeederIO feederIO) {
        this.feederIO = feederIO;
        this.feederMode = FeederModes.IDLE;
        this.feederState = FeederState.IDLE;
    }

    @Override
    public void periodic() {
        feederIO.updateInputs(feederInputs);
        // TODO: Add Logger.processInputs once auto-logged classes are generated
        // Logger.processInputs("Feeder", feederInputs);

        SmartDashboard.putString("Feeder Mode", feederMode.toString());
        SmartDashboard.putBoolean("Feeder Connected", feederInputs.feederConnected);
        SmartDashboard.putNumber("Feeder Voltage", feederInputs.appliedVoltage);
        SmartDashboard.putNumber("Feeder RPM", feederInputs.RPM);
        SmartDashboard.putNumber("Feeder Current", feederInputs.currentAmps);
        
        Logger.recordOutput("Feeder/Mode", feederMode);
        Logger.recordOutput("Feeder/State", feederState);

        switch (this.feederMode) {
            case IDLE:
                this.feederState = FeederState.IDLE;
                break;
            case FEEDER:
                this.feederState = FeederState.FEEDER;
                break;
            case REVERSE:
                this.feederState = FeederState.REVERSE;
                break;
        }
    }

    public void runFeederEnum(FeederModes feederMode) {
        this.feederMode = feederMode;
        feederIO.setFeederVoltage(feederMode.output);
    }

    public Command runFeederCommand(FeederModes feederMode) {
        return new RunCommand(() -> runFeederEnum(feederMode), this).withName("Feeder.runEnum");
    }
}
