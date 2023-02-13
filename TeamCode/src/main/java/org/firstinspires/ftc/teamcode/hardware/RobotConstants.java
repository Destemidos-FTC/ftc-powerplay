package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.autonomous.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.autonomous.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

@Config
public final class RobotConstants {

    // constraints do autonomo
    private static double ultra_factor = 3;
    public static final TrajectoryVelocityConstraint ULTRA_VEL_CONSTRAINT = getVelocityConstraint(
                    MAX_VEL * ultra_factor,
                    MAX_ANG_VEL * ultra_factor,
                    TRACK_WIDTH);

    public static final TrajectoryAccelerationConstraint ULTRA_ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL * ultra_factor);

    // Controles
    public static double DRIVER_CONTROLLER_X_AXIS_CORRECTION = 1.01;

    // Drivetrain
    public static double MAX_SPEED = 1.0;
    public static final double MECANUM_WHEELS_ANGLE = Math.PI / 4;
    public static final int CORE_HEX_TICKS = 288;
    public static final int HD_HEX_TICKS = 1120;
    public static int MOTOR_DIREITA_FRENTE_ID = 0;
    public static int MOTOR_DIREITA_TRAS_ID = 1;
    public static int MOTOR_ESQUERDA_FRENTE_ID = 2;
    public static int MOTOR_ESQUERDA_TRAS_ID = 3;

    // Braço
    public static double ARMS_POWER_SCALE = 0.7;
    public static double CENTRO_MAX_POWER = 0.6;
    public static double CENTRO_MIN_POWER = -1.0;

    // Servos
    public static double GARRA_A_POSITION_CLOSED = 0.85;
    public static double GARRA_A_POSITION_OPEN = 0.6;

    public static double GARRA_B_POSITION_CLOSED = 0.5;
    public static double GARRA_B_POSITION_OPEN = 0.15;

    public static double HAND_CLOSED_POSITION = 0.78;
    public static double HAND_MAX_POSITION = 0.4;

}
