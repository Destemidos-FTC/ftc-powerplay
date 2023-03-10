package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.config.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.math.parameters.FeedfowardCoeficients;

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

    // Drivetrain
    public static final double CORE_HEX_TICKS_PER_REV = 288.0;
    public static final double HD_HEX_40_TICKS_PER_REV = 1120.0;
    public static final double MECANUM_WHEELS_ANGLE = Math.PI / 4;

    // Braço
    public static PIDFCoefficients ARM_POSITION_PID = new PIDFCoefficients(0,0,0,0.5);
    public static PIDFCoefficients FOREARM_POSITION_PID = new PIDFCoefficients(0,0,0,0.5);

    public static double ARMS_POWER_SCALE = 0.6;
    public static double FOREARM_POWER_SCALE = 0.4;

    public static int ARM_GROUND_GOAL = 0;
    public static int ARM_LOW_GOAL = 400;
    public static int ARM_MEDIUM_GOAL = 700;
    public static int ARM_HIGH_GOAL = 1000;

    public static int FOREARM_CLOSED = 0;
    public static int FOREARM_LOW_GOAL = -75;
    public static int FOREARM_MEDIUM_GOAL = -120;

    // Servos
    public static final PwmControl.PwmRange MAX_SERVO_RANGE = new PwmControl.PwmRange(500, 2500, 18000);
    public static double GRIPPER_OPEN_POSITION = 0.3;
    public static double GRIPPER_CLOSED_POSITION = 0.0;

    // Outtake
    public static PIDFCoefficients OUTTAKE_PID = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
    public static FeedfowardCoeficients OUTTAKE_FEEDFORWARD = new FeedfowardCoeficients(0, 0, 0);

    // Intake
    public static PIDFCoefficients INTAKE_PID = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
    public static FeedfowardCoeficients INTAKE_FEEDFORWARD = new FeedfowardCoeficients(0, 0, 0);
}
