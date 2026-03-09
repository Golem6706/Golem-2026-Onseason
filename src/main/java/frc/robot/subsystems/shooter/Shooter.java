package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterContants.MAX_SHOOTER_RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private final SlewRateLimiter velocitySlewRateLimiter;

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

        velocitySlewRateLimiter = new SlewRateLimiter(ShooterContants.SHOOTER_ACCELERATION_RPM_PER_SEC, -12000, 0.0);

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

    private void setShooterTargetVelocity(double rpm) {
        if (rpm < 0.0 || rpm > ShooterContants.MAX_SHOOTER_RPM)
            System.out.println("Warning: Attempted to set shooter target RPM to " + rpm
                    + ", which is out of bounds. Clamping to valid range.");

        this.shooterTargetRPM = Math.max(0.0, Math.min(rpm, ShooterContants.MAX_SHOOTER_RPM));
    }

    /**
     * Gets the target shooter velocity in RPM.
     *
     * @return the target shooter velocity in RPM
     */
    public double getShooterTargetRPM() {
        return shooterTargetRPM;
    }

    public double getShooterCurrentRPM() {
        return velocitySlewRateLimiter.lastValue();
    }

    public double getShooterActualRPM() {
        if (inputs.shooterMotorsVelocityRPM == null || inputs.shooterMotorsVelocityRPM.length == 0) return 0.0;

        double sum = 0.0;
        for (double velocity : inputs.shooterMotorsVelocityRPM) {
            sum += velocity;
        }
        return sum / inputs.shooterMotorsVelocityRPM.length;
    }

    /**
     * Checks if the shooter is at the target velocity within the tolerance.
     *
     * @return true if the shooter is at the target velocity, false otherwise
     */
    public boolean isShooterAtVelocityReference() {
        return Math.abs(getShooterCurrentRPM() - getShooterTargetRPM()) <= ShooterContants.SHOOTER_VELOCITY_TOLERANCE_RPM;
    }

    private void executeControlLoop() {
        double shooterNextVelocity = velocitySlewRateLimiter.calculate(shooterTargetRPM);
        io.setShooterVelocity(shooterNextVelocity);
    }

    /**
     * Creates a command that stops all shooter and feeder motors.
     *
     * @return the command
     */
    @Override
    public Command idle() {
        return run(() -> {
            io.setShooterMotorsVoltage(0.0);
            io.setFeederMotorsVoltage(0.0);
            velocitySlewRateLimiter.reset(0.0);
            this.shooterTargetRPM = 0.0;
        }).ignoringDisable(true);
    }

    public Command runSetPoint(DoubleSupplier shooterRPMSupplier, DoubleSupplier feederVoltageSupplier) {
        return startRun(() -> velocitySlewRateLimiter.reset(getShooterActualRPM()),
            () -> {
            io.setFeederMotorsVoltage(feederVoltageSupplier.getAsDouble());

            this.shooterTargetRPM = shooterRPMSupplier.getAsDouble();
            if (this.shooterTargetRPM < 0.0 || this.shooterTargetRPM > MAX_SHOOTER_RPM)
                System.out.println("Setting shooter RPM" + this.shooterTargetRPM + " which is out of bound, clampping...");
            this.shooterTargetRPM = MathUtil.clamp(this.shooterTargetRPM, 0.0, MAX_SHOOTER_RPM);

            executeControlLoop();
        });
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
}
