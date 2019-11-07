package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class DriveConstants {

    /**
     * non-final fields can be edited through the dashboard (connect to robot's WiFi Direct network
     * and navigate to http://192.168.49.1:8080/dash).  Changes must be saved here to be permanent,
     * changes made in the dashboard to not persist
     */
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 1; // wheel speed / motor speed
    public static double TRACK_WIDTH = 13.5;
    public static double WHEEL_BASE = 15;

    /**
     * If the drive motors do not have encoders or they are not being used for velocity control,
     * tune these values empirically, otherwise they are fine as is
     */
    public static double kV = 1.0 / rpmToVelocity(getMaxRpm());
    public static double kA = 0;
    public static double kStatic = 0;

    /**
     * These constraintes should not exceed ~80% of the robot's actual capability.  Start small and
     * increase when everything is working properly.  Velocity and acceleration are required, jerk
     * is optional.  Jerk = 0 forces acceleration limited profiling
     */
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180), Math.toRadians(180), 0.0
    );

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm){
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60;
    }

    public static double getMaxRpm(){
        return MOTOR_CONFIG.getMaxRPM();
    }
}
