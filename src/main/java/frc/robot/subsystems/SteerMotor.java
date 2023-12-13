package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.utils.Gains;


public class SteerMotor {
    // Physical.
    private static final int CURRENT_LIMIT_AMPS = 30;

    // PID.
    private static final int PID_SLOT = 0;
    private static final Gains PID_GAINS = new Gains(20.0, 0.12, 1.0);

    // Smart motion.
    private static final double MAX_VELOCITY_RPM = 11_000.0;
    private static final double MIN_VELOCITY_RPM = 0.0;
    private static final double MAX_ACCELERATION_RPM_PER_SEC = MAX_VELOCITY_RPM * 2.0;
    private static final double ALLOWED_ERROR_ROTATIONS = 0.05;

    // Motor and PID controller.
    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_PIDController;

    // Encoders.
    private final AbsoluteEncoder m_absEncoder;

    
    public SteerMotor(int canID, double wheelZeroOffsetDegrees, boolean invertMotor) {
        // Motor.
        m_motor = new CANSparkMax(canID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
        m_motor.setIdleMode(IdleMode.kBrake);  // Idle brake mode for accurate steering.
        m_motor.setInverted(invertMotor);

        // Abs encoder.
        m_absEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
        m_absEncoder.setPositionConversionFactor(360.0);  // Use degrees for position.
        m_absEncoder.setZeroOffset(wheelZeroOffsetDegrees);

        // PID controller.
        m_PIDController = m_motor.getPIDController();
        m_PIDController.setFeedbackDevice(m_absEncoder);

        // Set PID wrapping (-180 to 180 degrees).
        m_PIDController.setPositionPIDWrappingEnabled(true);
        m_PIDController.setPositionPIDWrappingMinInput(-180.0);
        m_PIDController.setPositionPIDWrappingMaxInput(180.0);

        // Config PIDs and smart motion.
        PID_GAINS.setGains(m_PIDController, PID_SLOT);
        Gains.configSmartMotion(
            m_PIDController,
            MAX_VELOCITY_RPM,
            MIN_VELOCITY_RPM,
            MAX_ACCELERATION_RPM_PER_SEC, 
            ALLOWED_ERROR_ROTATIONS,
            PID_SLOT
        );
    }

    public Rotation2d getPositionRotation2d() {
        double positionDegrees = m_absEncoder.getPosition();
        return Rotation2d.fromDegrees(positionDegrees);
    }

    public void setTargetPositionRotation2d(Rotation2d targetPositionRotation2d) {
        double targetPositionDegrees = targetPositionRotation2d.getDegrees();
        m_PIDController.setReference(targetPositionDegrees, ControlType.kPosition);
    }
}
