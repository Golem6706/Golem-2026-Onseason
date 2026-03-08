package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterContants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX[] shooterMotors;
    private final StatusSignal<Current>[] shooterMotorCurrents;
    private final StatusSignal<Voltage>[] shooterMotorOutputVoltages;
    private final StatusSignal<AngularVelocity>[] shooterMotorVelocities;

    private final TalonFX[] feederMotors;
    private final StatusSignal<Current>[] feederMotorCurrents;
    private final StatusSignal<Voltage>[] feederMotorOutputVoltages;
    private final StatusSignal<AngularVelocity>[] feederMotorVelocities;

    private final VoltageOut voltageOut = new VoltageOut(Volts.zero());
    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

    public ShooterIOReal() {
        int shooterCount = SHOOTERHARDWARE_CONSTANTS.shooterMotorIDs().length;
        int feederCount = SHOOTERHARDWARE_CONSTANTS.feederMotorIDs().length;

        shooterMotors = new TalonFX[shooterCount];
        shooterMotorCurrents = new StatusSignal[shooterCount];
        shooterMotorOutputVoltages = new StatusSignal[shooterCount];
        shooterMotorVelocities = new StatusSignal[shooterCount];

        feederMotors = new TalonFX[feederCount];
        feederMotorCurrents = new StatusSignal[feederCount];
        feederMotorOutputVoltages = new StatusSignal[feederCount];
        feederMotorVelocities = new StatusSignal[feederCount];

        // Configure shooter motors
        for (int i = 0; i < shooterCount; i++) {
            int motorID = SHOOTERHARDWARE_CONSTANTS.shooterMotorIDs()[i];
            boolean inverted = SHOOTERHARDWARE_CONSTANTS.shooterMotorInverted()[i];

            shooterMotors[i] = new TalonFX(motorID);

            TalonFXConfiguration config = new TalonFXConfiguration()
                    .withCurrentLimits(new CurrentLimitsConfigs()
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(SHOOTER_MOTORS_CURRENT_LIMIT))
                    .withMotorOutput(new MotorOutputConfigs()
                            .withInverted(
                                    inverted
                                            ? InvertedValue.Clockwise_Positive
                                            : InvertedValue.CounterClockwise_Positive)
                            .withNeutralMode(NeutralModeValue.Coast))
                    .withSlot0(SHOOTER_VELOCITY_GAINS);
            shooterMotors[i].getConfigurator().apply(config);

            shooterMotorCurrents[i] = shooterMotors[i].getSupplyCurrent();
            shooterMotorOutputVoltages[i] = shooterMotors[i].getMotorVoltage();
            shooterMotorVelocities[i] = shooterMotors[i].getVelocity();

            BaseStatusSignal.setUpdateFrequencyForAll(
                    100, shooterMotorCurrents[i], shooterMotorOutputVoltages[i], shooterMotorVelocities[i]);

            shooterMotors[i].optimizeBusUtilization();
        }

        // Configure feeder motors
        for (int i = 0; i < feederCount; i++) {
            int motorID = SHOOTERHARDWARE_CONSTANTS.feederMotorIDs()[i];
            boolean inverted = SHOOTERHARDWARE_CONSTANTS.feederMotorInverted()[i];

            feederMotors[i] = new TalonFX(motorID);

            TalonFXConfiguration config = new TalonFXConfiguration()
                    .withCurrentLimits(new CurrentLimitsConfigs()
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(FEEDER_MOTORS_CURRENT_LIMIT))
                    .withMotorOutput(new MotorOutputConfigs()
                            .withInverted(
                                    inverted
                                            ? InvertedValue.Clockwise_Positive
                                            : InvertedValue.CounterClockwise_Positive)
                            .withNeutralMode(NeutralModeValue.Coast))
                    .withSlot0(FEEDER_VELOCITY_GAINS);
            feederMotors[i].getConfigurator().apply(config);

            feederMotorCurrents[i] = feederMotors[i].getSupplyCurrent();
            feederMotorOutputVoltages[i] = feederMotors[i].getMotorVoltage();
            feederMotorVelocities[i] = feederMotors[i].getVelocity();

            BaseStatusSignal.setUpdateFrequencyForAll(
                    100, feederMotorCurrents[i], feederMotorOutputVoltages[i], feederMotorVelocities[i]);

            feederMotors[i].optimizeBusUtilization();
        }
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        int shooterCount = shooterMotors.length;
        if (inputs.shootersConnected.length != shooterCount) {
            inputs.shootersConnected = new boolean[shooterCount];
            inputs.shooterMotorsVelocityRPM = new double[shooterCount];
        }
        if (inputs.feedersConnected.length != feederMotors.length) {
            inputs.feedersConnected = new boolean[feederMotors.length];
            inputs.feederMotorsVelocityRPM = new double[feederMotors.length];
        }

        double shooterTotalVolts = 0.0;
        double shooterTotalCurrent = 0.0;

        for (int i = 0; i < shooterCount; i++) {
            boolean connected = BaseStatusSignal.refreshAll(
                            shooterMotorCurrents[i], shooterMotorOutputVoltages[i], shooterMotorVelocities[i])
                    .isOK();

            inputs.shootersConnected[i] = connected;

            if (connected) {
                shooterTotalVolts += shooterMotorOutputVoltages[i].getValueAsDouble();
                shooterTotalCurrent += shooterMotorCurrents[i].getValueAsDouble();
                inputs.shooterMotorsVelocityRPM[i] = shooterMotorVelocities[i].getValueAsDouble() * 60.0;
            }
        }

        inputs.shooterMotorsAverageVolts = shooterTotalVolts / shooterCount;
        inputs.shooterMotorsTotalCurrentAmps = shooterTotalCurrent;

        int feederCount = feederMotors.length;
        double feederTotalVolts = 0.0;
        double feederTotalCurrent = 0.0;

        for (int i = 0; i < feederCount; i++) {
            boolean connected = BaseStatusSignal.refreshAll(
                            feederMotorCurrents[i], feederMotorOutputVoltages[i], feederMotorVelocities[i])
                    .isOK();

            inputs.feedersConnected[i] = connected;

            if (connected) {
                feederTotalVolts += feederMotorOutputVoltages[i].getValueAsDouble();
                feederTotalCurrent += feederMotorCurrents[i].getValueAsDouble();
                inputs.feederMotorsVelocityRPM[i] = feederMotorVelocities[i].getValueAsDouble() * 60.0;
            }
        }

        inputs.feederMotorsAverageVolts = feederTotalVolts / feederCount;
        inputs.feederMotorsTotalCurrentAmps = feederTotalCurrent;
    }

    @Override
    public void setShooterMotorsVoltage(double volts) {
        shooterMotors[0].setControl(voltageOut.withOutput(volts));
        for (int i = 1; i < shooterMotors.length; i++) {
            shooterMotors[i].setControl(new Follower(shooterMotors[0].getDeviceID(), MotorAlignmentValue.Aligned));
        }
    }

    @Override
    public void setFeederMotorsVoltage(double volts) {
        feederMotors[0].setControl(voltageOut.withOutput(volts));
        for (int i = 1; i < feederMotors.length; i++) {
            feederMotors[i].setControl(new Follower(feederMotors[0].getDeviceID(), MotorAlignmentValue.Aligned));
        }
    }

    @Override
    public void setShooterVelocity(double rpm) {
        double rps = rpm / 60.0;
        shooterMotors[0].setControl(shooterVelocityRequest.withVelocity(rps));
        for (int i = 1; i < shooterMotors.length; i++) {
            shooterMotors[i].setControl(new Follower(shooterMotors[0].getDeviceID(), MotorAlignmentValue.Aligned));
        }
    }
}
