package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.AlertsManager;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    // Hardware interface
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;

    // SysId routine for shooter characterization
    private final SysIdRoutine sysId;

    // Shooter velocity tracking
    private double shooterTargetRPM = 0.0;
    private double currentShooterVelocityRPM = 0.0;
    private double profiledTargetRPM = 0.0; // Trapezoidal profile target

    // Trapezoidal profile state
    private double lastProfileTime = 0.0;

    // Dynamic alerts based on motor count
    private Alert[] shooterMotorAlerts;
    private Alert[] feederMotorAlerts;

    public Shooter(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();

        // Initialize alerts arrays (will be populated in periodic based on actual motor count)
        shooterMotorAlerts = new Alert[0];
        feederMotorAlerts = new Alert[0];

        // Configure SysId routine for shooter
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runShooterCharacterization(voltage.in(Volts)), null, this));
    }

    public boolean hardwareOK() {
        // Check all shooter motors
        if (inputs.shootersConnected != null) {
            for (boolean connected : inputs.shootersConnected) {
                if (!connected) return false;
            }
        }

        // Check all feeder motors
        if (inputs.feedersConnected != null) {
            for (boolean connected : inputs.feedersConnected) {
                if (!connected) return false;
            }
        }

        return true;
    }

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Initialize or update alerts based on current motor count
        updateAlerts();

        if (DriverStation.isDisabled()) executeIdle();

        // Update trapezoidal velocity profile
        updateTrapezoidalProfile();

        // Update alerts
        if (shooterMotorAlerts.length > 0 && inputs.shootersConnected != null) {
            for (int i = 0; i < Math.min(shooterMotorAlerts.length, inputs.shootersConnected.length); i++) {
                if (shooterMotorAlerts[i] != null) {
                    shooterMotorAlerts[i].set(!inputs.shootersConnected[i]);
                }
            }
        }

        if (feederMotorAlerts.length > 0 && inputs.feedersConnected != null) {
            for (int i = 0; i < Math.min(feederMotorAlerts.length, inputs.feedersConnected.length); i++) {
                if (feederMotorAlerts[i] != null) {
                    feederMotorAlerts[i].set(!inputs.feedersConnected[i]);
                }
            }
        }

        // Log motor connections
        // Note: Already logged via inputs in processInputs

        // Log aggregated data
        // Note: Already logged via inputs in processInputs

        // Log velocity data and at-reference status
        // Note: AverageVelocityRPM already logged via inputs in processInputs
        if (inputs.shooterMotorsVelocityRPM != null && inputs.shooterMotorsVelocityRPM.length > 0) {
            double averageVelocity = calculateAverageVelocity(inputs.shooterMotorsVelocityRPM);
            currentShooterVelocityRPM = averageVelocity;
        }

        // Log at reference status
        Logger.recordOutput("Shooter/AtVelocityReference", isShooterAtVelocityReference());
        Logger.recordOutput("Shooter/TargetVelocityRPM", shooterTargetRPM);

        // Note: Feeder velocity already logged via inputs
    }

    private void updateAlerts() {
        // Create alerts for shooter motors if needed
        if (inputs.shootersConnected != null && shooterMotorAlerts.length != inputs.shootersConnected.length) {
            shooterMotorAlerts = new Alert[inputs.shootersConnected.length];
            for (int i = 0; i < shooterMotorAlerts.length; i++) {
                shooterMotorAlerts[i] =
                        AlertsManager.create("ShooterMotor" + (i + 1) + " hardware detected!", AlertType.kError);
            }
        }

        // Create alerts for feeder motors if needed
        if (inputs.feedersConnected != null && feederMotorAlerts.length != inputs.feedersConnected.length) {
            feederMotorAlerts = new Alert[inputs.feedersConnected.length];
            for (int i = 0; i < feederMotorAlerts.length; i++) {
                feederMotorAlerts[i] =
                        AlertsManager.create("FeederMotor" + (i + 1) + " hardware detected!", AlertType.kError);
            }
        }
    }

    private double calculateAverageVelocity(double[] velocities) {
        if (velocities == null || velocities.length == 0) return 0.0;

        double sum = 0.0;
        int count = 0;
        for (double velocity : velocities) {
            sum += velocity;
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    private void setShooterMotorVolts(double shooterMotorVolts) {
        if (!hardwareOK()) shooterMotorVolts = 0;
        io.setShooterMotorsVoltage(shooterMotorVolts);
    }

    private void setFeederMotorVolts(double feederMotorVolts) {
        if (!hardwareOK()) feederMotorVolts = 0;
        io.setFeederMotorsVoltage(feederMotorVolts);
    }

    private void setShooterVelocity(double rpm) {
        if (!hardwareOK()) rpm = 0;
        shooterTargetRPM = rpm;
        // The trapezoidal profile is applied in periodic()
        io.setShooterVelocity(0); // Will be updated by profile in periodic
    }

    /** Updates the trapezoidal velocity profile. Call this in periodic() to apply the profile. */
    private void updateTrapezoidalProfile() {
        double dt = 0.02; // 20ms period (50Hz)

        // Calculate the difference between target and current
        double error = shooterTargetRPM - profiledTargetRPM;

        // Calculate max change based on acceleration limit (12000 rpm/s)
        double maxChange = ShooterContants.SHOOTER_ACCELERATION_RPS * 60.0 * dt; // Convert to RPM change per cycle

        // Apply trapezoidal profile - change velocity gradually
        if (Math.abs(error) <= maxChange) {
            // Close enough, go directly to target
            profiledTargetRPM = shooterTargetRPM;
        } else {
            // Ramp toward target at acceleration limit
            if (error > 0) {
                profiledTargetRPM += maxChange;
            } else {
                profiledTargetRPM -= maxChange;
            }
        }

        // Apply the profiled velocity to the motor
        io.setShooterVelocity(profiledTargetRPM);
    }

    /** Returns the current shooter velocity in RPM. */
    public double getShooterVelocityRPM() {
        return currentShooterVelocityRPM;
    }

    /** Returns the shooter target velocity in RPM. */
    public double getShooterTargetRPM() {
        return shooterTargetRPM;
    }

    /** Checks if the shooter is at the target velocity within the tolerance. */
    public boolean isShooterAtVelocityReference() {
        return Math.abs(currentShooterVelocityRPM - shooterTargetRPM) <= ShooterContants.SHOOTER_VELOCITY_TOLERANCE_RPM;
    }

    private void executeIdle() {
        setShooterMotorVolts(0.0);
        setFeederMotorVolts(0.0);
    }

    public Command runShooter(double shooterMotorVolts) {
        return run(() -> setShooterMotorVolts(shooterMotorVolts));
    }

    public Command runFeeder(double feederMotorVolts) {
        return run(() -> setFeederMotorVolts(feederMotorVolts));
    }

    public Command runShooterVelocity(double rpm) {
        return run(() -> setShooterVelocity(rpm));
    }

    public Command runSetPoint(double shooterRPM, double feederRPM) {
        return runShooterVelocity(shooterRPM).alongWith(runFeeder(feederRPM));
    }

    public Command runIdle() {
        return runShooter(0).alongWith(runFeeder(0));
    }

    /**
     * Runs the shooter motor with the specified voltage for SysId characterization.
     *
     * @param volts the voltage to apply to the shooter motor
     */
    public void runShooterCharacterization(double volts) {
        if (!hardwareOK()) volts = 0;
        io.setShooterMotorsVoltage(volts);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runShooterCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runShooterCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Creates a command that runs the shooter at a variable speed and controls the feeder. The feeder runs only when
     * the trigger is pressed AND the shooter is at the target velocity.
     *
     * @param rpmSupplier supplier for the target shooter RPM
     * @param triggerSupplier supplier for the shoot trigger (true to shoot)
     * @return the command
     */
    public Command shootCommand(DoubleSupplier rpmSupplier, BooleanSupplier triggerSupplier) {
        return run(() -> {
            double rpm = rpmSupplier.getAsDouble();
            setShooterVelocity(rpm);

            // Run feeder if triggered AND shooter is at reference velocity
            if (triggerSupplier.getAsBoolean() && isShooterAtVelocityReference()) {
                setFeederMotorVolts(ShooterContants.VOLTAGE_SETTINGS.feederMotorVolts()[0]);
            } else {
                setFeederMotorVolts(0.0);
            }
        });
    }

    /**
     * Creates a command that runs the shooter at a fixed speed and controls the feeder. The feeder runs only when the
     * trigger is pressed AND the shooter is at the target velocity.
     *
     * @param rpm target shooter RPM
     * @param triggerSupplier supplier for the shoot trigger (true to shoot)
     * @return the command
     */
    public Command shootCommand(double rpm, BooleanSupplier triggerSupplier) {
        return shootCommand(() -> rpm, triggerSupplier);
    }
}
