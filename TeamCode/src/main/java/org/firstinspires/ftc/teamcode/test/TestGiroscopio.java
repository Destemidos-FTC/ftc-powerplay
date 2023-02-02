package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.controllers.GyroController;
import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

@Autonomous(name = "Test Giroscopio", group = "Test")
public class TestGiroscopio extends LinearOpMode {
    private DestemidosBot robot;
    private GyroController gyroController;

    public static PIDFCoefficients GYRO_COEFFICIENTS= new PIDFCoefficients(0, 0, 0, 0);
    public static double targetAngle = 90;

    private double getAbsoluteAngle() {
        return robot.drivetrain.getSensorIMU().getRobotOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosBot(hardwareMap);
        gyroController = new GyroController(GYRO_COEFFICIENTS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        // giro de 90 graus por padrão
        double robotAngle = getAbsoluteAngle();
        double pid_output = gyroController.calculate(robotAngle + targetAngle, robotAngle);

        robot.drivetrain.setAllPower(pid_output);

        sleep(3000);
    }
}
