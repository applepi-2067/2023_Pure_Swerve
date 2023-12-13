package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;

import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;


public class DriveMotor {
    private final WPI_TalonFX m_motor;

    // Motor settings.
    private static final boolean ENABLE_CURRENT_LIMIT = true;
    private static final double CONTINUOUS_CURRENT_LIMIT_AMPS = 55.0;
    private static final double TRIGGER_THRESHOLD_LIMIT_AMPS = 60.0;
    private static final double TRIGGER_THRESHOLD_TIME_SECONDS = 0.5;

    private static final double PERCENT_DEADBAND = 0.001;
    
    // Conversion constants.
    private static final double TICKS_PER_REV = 2048.0;
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(4.0 / 2.0);

    private static final double GEAR_RATIO = (30.0 / 14.0) * (45.0 / 15.0);  // motor pinion -> gear, bevel gear pair.

    // PID.
    private static final int K_TIMEOUT_MS = 10;
    private static final int K_PID_LOOP = 0;

    private static final int K_VELOCITY_PID_SLOT = 0;
    private static final Gains VELOCITY_PID_GAINS = new Gains(0.025, 0.045, 1.0);

    private static final int K_POSITION_PID_SLOT = 1;
    private static final Gains POSITION_PID_GAINS = new Gains(0.0, 0.0, 0.0);  // TODO: tune gains.

    private static final int CRUISE_VELOCITY_TICKS_PER_100MS = (int) Conversions.RPMToTicksPer100ms(
        Conversions.metersPerSecondToRPM(
            Drivetrain.MAX_TRANSLATION_SPEED_METERS_PER_SEC,
            WHEEL_RADIUS_METERS
        ),
        TICKS_PER_REV
    );
    private static final int MAX_ACCEL_TICKS_PER_100MS_PER_SEC = CRUISE_VELOCITY_TICKS_PER_100MS * 2;


    public DriveMotor(int canID, boolean invertMotor) {
        // Motor.
        m_motor = new WPI_TalonFX(canID);
        m_motor.configFactoryDefault();

        m_motor.setNeutralMode(NeutralMode.Coast);  // Coast to avoid tipping.

        // Limit current going to motor.
        SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(
            ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT_AMPS,
            TRIGGER_THRESHOLD_LIMIT_AMPS, TRIGGER_THRESHOLD_TIME_SECONDS
        );
        m_motor.configSupplyCurrentLimit(talonCurrentLimit);

        // Config velocity and position control.
        m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, K_PID_LOOP, K_TIMEOUT_MS);
        m_motor.setSelectedSensorPosition(0.0);
        m_motor.configNeutralDeadband(PERCENT_DEADBAND, K_TIMEOUT_MS);
        m_motor.setInverted(invertMotor);

        VELOCITY_PID_GAINS.setGains(m_motor, K_VELOCITY_PID_SLOT, K_PID_LOOP, K_TIMEOUT_MS);

        POSITION_PID_GAINS.setGains(m_motor, K_POSITION_PID_SLOT, K_PID_LOOP, K_TIMEOUT_MS);
        Gains.configMotionMagic(m_motor, CRUISE_VELOCITY_TICKS_PER_100MS, MAX_ACCEL_TICKS_PER_100MS_PER_SEC, K_TIMEOUT_MS);
    }

    public void setTargetVelocityMetersPerSecond(double wheelVelocityMetersPerSecond) {
        m_motor.selectProfileSlot(K_VELOCITY_PID_SLOT, K_PID_LOOP);
        double velocityMetersPerSecond = wheelVelocityMetersPerSecond * GEAR_RATIO;

        double velocityRPM = Conversions.metersPerSecondToRPM(velocityMetersPerSecond, WHEEL_RADIUS_METERS);
        double velocityTicksPer100ms = Conversions.RPMToTicksPer100ms(velocityRPM, TICKS_PER_REV);
        m_motor.set(TalonFXControlMode.Velocity, velocityTicksPer100ms);
    }

    public double getVelocityMetersPerSecond() {
        double velocityTicksPer100ms = m_motor.getSelectedSensorVelocity(K_PID_LOOP);
        double velocityRPM = Conversions.ticksPer100msToRPM(velocityTicksPer100ms, TICKS_PER_REV);
        double velocityMetersPerSecond = Conversions.rpmToMetersPerSecond(velocityRPM, WHEEL_RADIUS_METERS);

        double wheelVelocityMetersPerSecond = velocityMetersPerSecond / GEAR_RATIO;
        return wheelVelocityMetersPerSecond;
    }

    public void setTargetPositionMeters(double meters) {
        m_motor.selectProfileSlot(K_POSITION_PID_SLOT, K_PID_LOOP);

        double ticks = Conversions.metersToTicks(meters, TICKS_PER_REV, GEAR_RATIO, WHEEL_RADIUS_METERS);
        m_motor.set(TalonFXControlMode.MotionMagic, ticks);
    }


    public double getPositionMeters() {
        double ticks = m_motor.getSelectedSensorPosition();
        double meters = Conversions.ticksToMeters(ticks, TICKS_PER_REV, GEAR_RATIO, WHEEL_RADIUS_METERS);
        return meters;
    }
}
