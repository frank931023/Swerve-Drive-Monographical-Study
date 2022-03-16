
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor.getEncoder().setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveMotor.getEncoder().setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningMotor.getEncoder().setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningMotor.getEncoder().setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        // Set the encoder convert constants so that we can work with meters and radians
        // instead of rotations(rpm)

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // Proportional term alone already does a good job of adjusting the wheel angle for turningMotor
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        // Set the pid controller to be continous and considers -math.pi and math.pi to
        // be the same point and automatically calculates the shortest route to the
        // setpoint

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition() {
        return turningMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angleDeg = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        double angleRad = Units.degreesToRadians(angleDeg);
        angleRad -= absoluteEncoderOffsetRad;
        // Subtract the offset to get the actual wheel angle
        return angleRad * (absoluteEncoderReversed ? -1.0 : 1.0);
        // Multiply by -1 if it's reversed
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // Add a deadband
        state = SwerveModuleState.optimize(state, getState().angle);
        // Optimize the angle setpoint so it would never have to move more than 90 deg
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // SScale the velocity down using the robot's max speed
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        // Use pid controller for turningMotor to calculate the output for the angle
        // setpoint and current position
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        // Send out some debug info
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}