package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertsManager;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    // Hardware interface
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;

    // Alerts
    private final Alert shooterMotor1HardwareFaultAlert;
    private final Alert shooterMotor2HardwareFaultAlert;
    private final Alert shooterMotor3HardwareFaultAlert;

    private final Alert feederMotor1HardwareFaultAlert;
    private final Alert feederMotor2HardwareFaultAlert;

    // private final Supplier<Pose2d> robotPoseSupplier;
    // private final Supplier<Rotation2d> shooterAngleSupplier;

    // public Shooter(ShooterIO io, Supplier<Pose2d> robotPoseSupplier, Supplier<Rotation2d> shooterAngleSupplier) {
    public Shooter(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();
        // this.robotPoseSupplier =  robotPoseSupplier;
        // this.shooterAngleSupplier = shooterAngleSupplier;

        this.shooterMotor1HardwareFaultAlert =
                AlertsManager.create("ShooterMotor1 hardware detected!", AlertType.kError);
        this.shooterMotor2HardwareFaultAlert =
                AlertsManager.create("ShooterMotor2 hardware detected!", AlertType.kError);
        this.shooterMotor3HardwareFaultAlert =
                AlertsManager.create("ShooterMotor3 hardware detected!", AlertType.kError);
        this.feederMotor1HardwareFaultAlert = AlertsManager.create("FeederMotor1 hardware detected!", AlertType.kError);
        this.feederMotor2HardwareFaultAlert = AlertsManager.create("FeederMotor2 hardware detected!", AlertType.kError);
    }

    public boolean hardwareOK() {
        return inputs.shooterMotor1Connected
                && inputs.shooterMotor2Connected
                && inputs.shooterMotor3Connected
                && inputs.feederMotor1Connected
                && inputs.feederMotor2Connected;
    }

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        // Update alerts
        shooterMotor1HardwareFaultAlert.set(!inputs.shooterMotor1Connected);
        shooterMotor2HardwareFaultAlert.set(!inputs.shooterMotor2Connected);
        shooterMotor3HardwareFaultAlert.set(!inputs.shooterMotor3Connected);

        feederMotor1HardwareFaultAlert.set(!inputs.feederMotor1Connected);
        feederMotor2HardwareFaultAlert.set(!inputs.feederMotor2Connected);

        Logger.recordOutput("ShooterMotor1/connected", inputs.shooterMotor1Connected);
        Logger.recordOutput("ShooterMotor2/connected", inputs.shooterMotor2Connected);
        Logger.recordOutput("ShooterMotor3/connected", inputs.shooterMotor3Connected);

        Logger.recordOutput("FeederMotor1/connected", inputs.feederMotor1Connected);
        Logger.recordOutput("FeederMotor2/connected", inputs.feederMotor2Connected);
    }

    // private void setVoltage(double shooterMotorsVolts, double feederMotorsVolts) {
    //     if(!hardwareOK())
    //         shooterMotorsVolts = feederMotorsVolts = 0.0;
    //     io.setShooterMotorsVoltage(shooterMotorsVolts);
    //     io.setFeederMotorsVoltage(feederMotorsVolts);
    // }

    private void setShooterMotorVolts(double shooterMotorVolts) {
        if (!hardwareOK()) shooterMotorVolts = 0;
        io.setShooterMotorsVoltage(shooterMotorVolts);
    }

    private void setFeederMotorVolts(double feederMotorVolts) {
        if (!hardwareOK()) feederMotorVolts = 0;
        io.setFeederMotorsVoltage(feederMotorVolts);
    }

    // public Command runVolts(double shooterMotorsVolts, double feederMotorVolts) {
    //     return run(() -> setVoltage(shooterMotorsVolts, feederMotorVolts));
    // }

    public Command runShooter(double shooterMotorVolts) {
        return run(() -> setShooterMotorVolts(shooterMotorVolts));
    }

    public Command runFeeder(double feederMotorVolts) {
        return run(() -> setFeederMotorVolts(feederMotorVolts));
    }

    public Command runIdle() {
        return runShooter(0).alongWith(runFeeder(0));
    }

    // public Command shooterSequence() {
    //     return Commands.sequence(
    //             runVolts(8, 0).withTimeout(2.0),
    //             runVolts(8, 4));
    // }

}
