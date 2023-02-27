package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.roadruneerquickstart.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.roadruneerquickstart.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Classe responsável por agrupar todas as configurações gerais
 * do robô em um só lugar, é recomendado o uso do FTC-Dashboard
 * para modificar e monitorar as mudanças destas informações em
 * tempo real
 */
@Config
public final class RobotConstants {

    // Hubs
    public static final int CONTROLHUB_ID = 0;
    public static final int EXPANSIONHUB_ID = 1;

    // Constraints específicas do Autônomo
    public static TrajectoryVelocityConstraint VEL_CONSTRAINT = DriveConstants.setVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = DriveConstants.setAccelerationConstraint(MAX_ACCEL);

    // Controles
    public static double DRIVER_CONTROLLER_X_AXIS_CORRECTION = 1.01;

    // Drivetrain
    public static final double CORE_HEX_TICKS_PER_REV = 288.0;
    public static final double ULTRAPLANETARY_TICKS_PER_REV = 288.0;
    public static final double HD_HEX_40_TICKS_PER_REV = 1120.0;
    public static final double MECANUM_WHEELS_ANGLE = Math.PI / 4;

    // Braço
    public static PIDFCoefficients ARM_POSITION_PID = new PIDFCoefficients(7, 0,4.3,0.0015);
    public static double ARMS_POWER_SCALE = 0.7;

    // Servos
    public static double GARRA_A_POSITION_CLOSED = 0.85;
    public static double GARRA_A_POSITION_OPEN = 0.6;

    public static double GARRA_B_POSITION_CLOSED = 0.5;
    public static double GARRA_B_POSITION_OPEN = 0.15;

    public static double HAND_CLOSED_POSITION = 0.78;
    public static double HAND_MAX_POSITION = 0.4;
}
