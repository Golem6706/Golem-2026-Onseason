package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;

    private final SysIdRoutine sysId;

    private double shooterTargetRPM = 0.0;
    private double currentShooterVelocityRPM = 0.0;

    private TrapezoidProfile velocityProfile;
    private TrapezoidProfile.State previousProfileState = new TrapezoidProfile.State(0.0, 0.0);

    private final Alert[] shooterMotorAlerts;
    private final Alert[] feederMotorAlerts;

    public Shooter(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();

        int shooterCount = ShooterContants.SHOOTERHARDWARE_CONSTANTS.shooterMotorIDs().length;
        int feederCount = ShooterContants.SHOOTERHARDWARE_CONSTANTS.feederMotorIDs().length;

        shooterMotorAlerts = new Alert[shooterCount];
        for (int i = 0; i < shooterCount; i++) {
            shooterMotorAlerts[i] = AlertsManager.create("ShooterMotor" + (i + 1) + " disconnected!", AlertType.kError);
        }

        feederMotorAlerts = new Alert[feederCount];
        for (int i = 0; i < feederCount; i++) {
            feederMotorAlerts[i] = AlertsManager.create("FeederMotor" + (i + 1) + " disconnected!", AlertType.kError);
        }

        velocityProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                ShooterContants.MAX_SHOOTER_RPM, ShooterContants.SHOOTER_ACCELERATION_RPM_PER_SEC));

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runShooterCharacterization(voltage.in(Volts)), null, this));
    }

    /**
     * Checks if all shooter and feeder motors are connected and functioning.
     *
     * @return true if all motors are connected, false otherwise
     */
    public boolean hardwareOK() {
        if (inputs.shootersConnected != null) {
            for (boolean connected : inputs.shootersConnected) {
                if (!connected) return false;
            }
        }

        if (inputs.feedersConnected != null) {
            for (boolean connected : inputs.feedersConnected) {
                if (!connected) return false;
            }
        }

        return true;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        updateAlerts();

        if (DriverStation.isDisabled()) executeIdle();

        updateTrapezoidalProfile();

        if (inputs.shooterMotorsVelocityRPM != null && inputs.shooterMotorsVelocityRPM.length > 0) {
            currentShooterVelocityRPM = calculateAverageVelocity(inputs.shooterMotorsVelocityRPM);
        }

        Logger.recordOutput("Shooter/AtVelocityReference", isShooterAtVelocityReference());
        Logger.recordOutput("Shooter/TargetVelocityRPM", shooterTargetRPM);
    }

    private void updateAlerts() {
        if (inputs.shootersConnected != null) {
            for (int i = 0; i < Math.min(shooterMotorAlerts.length, inputs.shootersConnected.length); i++) {
                shooterMotorAlerts[i].set(!inputs.shootersConnected[i]);
            }
        }

        if (inputs.feedersConnected != null) {
            for (int i = 0; i < Math.min(feederMotorAlerts.length, inputs.feedersConnected.length); i++) {
                feederMotorAlerts[i].set(!inputs.feedersConnected[i]);
            }
        }
    }

    private double calculateAverageVelocity(double[] velocities) {
        if (velocities == null || velocities.length == 0) return 0.0;

        double sum = 0.0;
        for (double velocity : velocities) {
            sum += velocity;
        }
        return sum / velocities.length;
    }

    private void setShooterMotorVolts(double shooterMotorVolts) {
        if (!hardwareOK()) shooterMotorVolts = 0;
        io.setShooterMotorsVoltage(shooterMotorVolts);
    }

    private void setFeederMotorVolts(double feederMotorVolts) {
        if (!hardwareOK()) feederMotorVolts = 0;
        io.setFeederMotorsVoltage(feederMotorVolts);
    }

    public void setFeederVoltage(double feederMotorVolts) {
        setFeederMotorVolts(feederMotorVolts);
    }

    private void setShooterVelocity(double rpm) {
        if (!hardwareOK()) rpm = 0;
        shooterTargetRPM = rpm;
        io.setShooterVelocity(0);
    }

    public void setShooterTargetRPM(double rpm) {
        setShooterVelocity(rpm);
    }

    private void updateTrapezoidalProfile() {
        double dt = 0.02;

        TrapezoidProfile.State goal = new TrapezoidProfile.State(shooterTargetRPM, 0.0);
        TrapezoidProfile.State nextState = velocityProfile.calculate(dt, previousProfileState, goal);

        previousProfileState = nextState;

        io.setShooterVelocity(nextState.position);
    }

    /**
     * Gets the current shooter velocity in RPM.
     *
     * @return the current shooter velocity in RPM
     */
    public double getShooterVelocityRPM() {
        return currentShooterVelocityRPM;
    }

    /**
     * Gets the target shooter velocity in RPM.
     *
     * @return the target shooter velocity in RPM
     */
    public double getShooterTargetRPM() {
        return shooterTargetRPM;
    }

    /**
     * Checks if the shooter is at the target velocity within the tolerance.
     *
     * @return true if the shooter is at the target velocity, false otherwise
     */
    public boolean isShooterAtVelocityReference() {
        return Math.abs(currentShooterVelocityRPM - shooterTargetRPM) <= ShooterContants.SHOOTER_VELOCITY_TOLERANCE_RPM;
    }

    private void executeIdle() {
        setShooterMotorVolts(0.0);
        setFeederMotorVolts(0.0);
    }

    /**
     * Creates a command that runs the shooter motors at a specified voltage.
     *
     * @param shooterMotorVolts the voltage to apply to the shooter motors
     * @return the command
     */
    public Command runShooter(double shooterMotorVolts) {
        return run(() -> setShooterMotorVolts(shooterMotorVolts));
    }

    /**
     * Creates a command that runs the feeder motors at a specified voltage.
     *
     * @param feederMotorVolts the voltage to apply to the feeder motors
     * @return the command
     */
    public Command runFeeder(double feederMotorVolts) {
        return run(() -> setFeederMotorVolts(feederMotorVolts));
    }

    /**
     * Creates a command that runs the shooter at a specified velocity using closed-loop control.
     *
     * @param rpm the target velocity in RPM
     * @return the command
     */
    public Command runShooterVelocity(double rpm) {
        return run(() -> setShooterVelocity(rpm));
    }

    /**
     * Creates a command that runs both shooter and feeder at specified velocities.
     *
     * @param shooterRPM the target shooter velocity in RPM
     * @param feederRPM the target feeder velocity in RPM (open-loop voltage is used)
     * @return the command
     */
    public Command runSetPoint(double shooterRPM, double feederRPM) {
        return runShooterVelocity(shooterRPM).alongWith(runFeeder(feederRPM));
    }

    /**
     * Creates a command that stops all shooter and feeder motors.
     *
     * @return the command
     */
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

    /**
     * Creates a command that runs a quasistatic SysId test in the specified direction.
     *
     * @param direction the direction for the quasistatic test
     * @return the command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runShooterCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /**
     * Creates a command that runs a dynamic SysId test in the specified direction.
     *
     * @param direction the direction for the dynamic test
     * @return the command
     */
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
