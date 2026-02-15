package frc.robot.subsystems.arm;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertsManager;

public class Arm extends SubsystemBase{
    //Hardware interface
    private final ArmIO io;
    private final ArmIO.ArmInputs inputs;

    //Controllers
    private final ArmFeedforward feedforwardController;
    private final PIDController feedbackController;
    private final TrapezoidProfile profile;

    //Alert
    private final Alert armHardwareFaultsAlert;
    private final Alert armNotCalibratedAlert;
    private final Alert armAbsoluteEncoderDisconnectedAlert;
    
    /** Debounce for hardware faults */
    private final Debouncer hardwareFaultDebouncer;
    /** Whether there is a hardware fault in the motor */
    private boolean hardwareFaultDetected;
    /** Whether the encoder has been calibrated since booting */
    private boolean encoderCalibrated;
    /** The offset of the relative encoder (relative - actual) */
    private Rotation2d relativeEncoderOffset;

    /** The current state */
    private TrapezoidProfile.State currentStateRad;

    /** The desired angle */
    private Angle setpoint;

    public Arm(ArmIO io) {
        this.io = io;
        inputs = new ArmIO.ArmInputs();

        this.feedforwardController = 
                new ArmFeedforward(PID_CONSTANTS.kS(), PID_CONSTANTS.kG(), PID_CONSTANTS.kV(), PID_CONSTANTS.kA());
        this.feedbackController = new PIDController(PID_CONSTANTS.kP(), 0, 0);
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                PID_CONSTANTS.VELOCTIY_CONSTRAIN().in(RadiansPerSecond),
                PID_CONSTANTS.ACCELERATION_CONSTRAIN().in(RadiansPerSecondPerSecond)));
        this.armHardwareFaultsAlert = AlertsManager.create("Arm hardware faults detected!", AlertType.kError);
        this.armNotCalibratedAlert = 
                AlertsManager.create("Arm encoder not calibrated! (Arm fault highly likely)", AlertType.kError);
        this.armAbsoluteEncoderDisconnectedAlert = AlertsManager.create(
                "Arm encoder calibrated, but absolute encoder disconnected. (Check it post-match).", AlertType.kWarning);
        this.hardwareFaultDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);

        this.armHardwareFaultsAlert.set(false);
        this.armNotCalibratedAlert.set(false);
        this.armAbsoluteEncoderDisconnectedAlert.set(false);

        currentStateRad = new TrapezoidProfile.State(
                HARDWARE_CONSTANTS.ARM_UPPER_HARD_LIMIT().in(Radians), 0.0);
        setpoint = ARM_SETPOINT_ANGLE;

        hardwareFaultDetected = false;
        encoderCalibrated = false;

        /**
         * Calibrate the relative encoder.
         * Assumes that the arm starts at upper limit during boot.
         * The calibration will be overwritten if the absolute encoder is available.
         * Offset = Relative Angle - Actual Angle, where Relative Angle is 0 when booting.
         */
        relativeEncoderOffset = Rotation2d.kZero.minus(new Rotation2d(HARDWARE_CONSTANTS.ARM_UPPER_HARD_LIMIT()));

        io.setArmMotorBrake(true);
        io.setIntakeMotorBrake(true);
    }

    /**
     * Calibrates the relative encoder offset using the absolute encoder reading
     * real = relative - offset, so offset = relative - real
     */
    private void calibrateEncoders(Rotation2d absoluteEncoderAngle) {
        relativeEncoderOffset = Rotation2d.fromRadians(
                        inputs.relativeEncoderAngledRad / HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION())
                .minus(absoluteEncoderAngle);
        encoderCalibrated = true;
    }

    /**
     * Sets the arm motor to idle
     * <p><b>Note: This does not unlock the motors if they are set to brake mode
     */
    private void executeIdle() {
        io.setArmMotorOutput(Volts.zero());
        currentStateRad = new TrapezoidProfile.State(getArmAngle().getRadians(), 0);
        previousVelocityRadPerSec = 0.0;
    }
    
    private double previousVelocityRadPerSec = 0.0;
 
    /**@return the measured arm angle, where zero is horizontally forward */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromRadians(inputs.relativeEncoderAngledRad / HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION())
                .minus(relativeEncoderOffset);
    }
    public double getVelocityRadPerSec() {
        return inputs.relativeEncoderVelocityRadPerSec / HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION();
    }
    public Rotation2d getProfileCurrentState() {
        return Rotation2d.fromRadians(currentStateRad.position);
    }

    /** Sets the brake mode of the arm motor. */
    public void setArmMotorBrake(boolean brakeModeEnable) {
        io.setArmMotorBrake(brakeModeEnable);
    }
}
