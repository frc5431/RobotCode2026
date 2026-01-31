package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerModes;
import frc.robot.Constants.IndexerConstants.IndexerState;
import frc.team5431.titan.core.subsystem.CTREMechanism;
import lombok.Getter;
import lombok.Setter;

public class Indexer extends CTREMechanism {

    @Getter @Setter private IndexerModes indexerModes;
    @Getter @Setter private IndexerState indexerState;

    private boolean attached;
    private TalonFX motor;

    public static class IndexerConfig extends Config {
        public IndexerConfig() {
            super("Indexer", Constants.CANBUS);

            configNeutralBrakeMode(IndexerConstants.breakType);
            configStatorCurrentLimit(IndexerConstants.stallLimit);
            configForwardSoftLimit(IndexerConstants.maxForwardOutput, true);
            configReverseSoftLimit(IndexerConstants.maxReverseOutput, true);
            configPIDGains(IndexerConstants.p, IndexerConstants.i, IndexerConstants.d);
            configPeakOutput(IndexerConstants.maxForwardOutput, IndexerConstants.maxReverseOutput);
            configGearRatio(IndexerConstants.gearRatio);
            configMotorInverted(IndexerConstants.invert);
        }
    }

    public Indexer(TalonFX motor, boolean attached, IndexerConfig config) {
        super(motor, attached, config);
        this.attached = attached;
        this.motor = motor;
        this.indexerModes = IndexerModes.IDLE;
        this.indexerState = IndexerState.IDLE;

        config.applyTalonConfig(motor);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Indexer Mode", getIndexerModes().toString());
        

        switch (this.indexerModes) {
            case IDLE:
                setIndexerState(IndexerState.IDLE);
                break;
            case INDEXER:
                setIndexerState(IndexerState.INDEXER);
                break;
            case REVERSE:
                setIndexerState(IndexerState.REVERSE);
                break;
        }

    }

    public Command runIndexerCommand(IndexerModes indexerModes) {
        return new RunCommand(() -> setPercentOutput(indexerModes.output), this).withName("Indexer.runEnum");
    }
}
