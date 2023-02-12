package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class RobotConstants {

    // Controles
    public static double DRIVER_CONTROLLER_X_AXIS_CORRECTION = 1.01;

    // Drivetrain
    public static final int MOTOR_DIREITA_FRENTE_ID = 0;
    public static final int MOTOR_DIREITA_TRAS_ID = 1;
    public static final int MOTOR_ESQUERDA_FRENTE_ID = 2;
    public static final int MOTOR_ESQUERDA_TRAS_ID = 3;

    public static final int CORE_HEX_TICKS = 288;
    public static final int HD_HEX_TICKS = 1120;
    public static final double MECANUM_WHEELS_ANGLE = Math.PI / 4;
    public static double MAX_SPEED = 1.0;

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
