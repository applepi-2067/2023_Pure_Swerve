package frc.robot.subsystems;


import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.constants.RobotMap;


public class SwerveModule {
    // Reported abs encoder position at wheel zero.
    // TODO: tune wheel zero offsets.
    private static final double[] STEER_WHEEL_ZERO_OFFSET_DEGREES = {240.52, 157.41, 223.30, 27.93};

    // Motor inversions.
    // TODO: check drive and steer motor inversions.
    private static final boolean[] INVERT_DRIVE_MOTORS = {false, false, false, false};
    private static final boolean[] INVERT_STEER_MOTORS = {false, false, false, false};

    // Motors.
    private final DriveMotor m_driveMotor;
    private final SteerMotor m_steerMotor;

    private final int location;

    
    public SwerveModule(int location) {
        this.location = location;

        // Create motors.
        m_driveMotor = new DriveMotor(
            RobotMap.canIDs.Drivetrain.DRIVE[location],
            INVERT_DRIVE_MOTORS[location]
            );
        m_steerMotor = new SteerMotor(
            RobotMap.canIDs.Drivetrain.STEER[location],
            STEER_WHEEL_ZERO_OFFSET_DEGREES[location],
            INVERT_STEER_MOTORS[location]
        );
    }

    public void setTargetState(SwerveModuleState targetState) {
        // Do nothing if target state is to not move.
        if (Math.abs(targetState.speedMetersPerSecond) < 0.001) {
            m_driveMotor.setTargetVelocityMetersPerSecond(0.0);
            return;
        }

        // Optimize state, inverting steer and drive rotations for shortest turn.
        targetState = SwerveModuleState.optimize(targetState, m_steerMotor.getPositionRotation2d());

        // Set steer and drive motors to targets.
        m_steerMotor.setTargetPositionRotation2d(targetState.angle);
        m_driveMotor.setTargetVelocityMetersPerSecond(targetState.speedMetersPerSecond);
    }

    public SwerveModuleState getState() {
        SwerveModuleState state = new SwerveModuleState(
            m_driveMotor.getVelocityMetersPerSecond(),
            m_steerMotor.getPositionRotation2d()
        );
        return state;
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition position = new SwerveModulePosition(
            m_driveMotor.getPositionMeters(),
            m_steerMotor.getPositionRotation2d()
        );
        return position;
    }

    public String toString() {
        String description = "Loc " + location + ":    ";
        description += getState().toString();
        return description;
    }
}
