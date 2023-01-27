package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

@Config
@Autonomous(name = "TestPIDF", group = "Test")
@Disabled
public class TestPIDF extends LinearOpMode {
    private DestemidosBot robot;
    private double[] positions = new double[4];
    public static PIDController controller;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosBot(hardwareMap);
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        // (ramalho) é bem provável que isso não funcione, mas ok
        for (int i = 0; i < robot.drivetrain.getMotors().size(); ++i ) {
            positions[i] = robot.drivetrain.getMotor(i).getCurrentPosition();
        }

        // por enquanto, só vou usar a posição do primeiro
        double output = controller.calculate(10, positions[0] );
        robot.drivetrain.setAllPower(output);
    }
}
