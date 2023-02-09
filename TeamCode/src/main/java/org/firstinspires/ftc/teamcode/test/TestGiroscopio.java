package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.controllers.GyroController;
import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

@Config
@Autonomous(name = "Test Giroscopio", group = "Test")
public class TestGiroscopio extends LinearOpMode {
    private DestemidosBot robot;
    private GyroController gyroController;

    public static PIDFCoefficients GYRO_COEFFICIENTS= new PIDFCoefficients(2.03, 0, 0, 0);
    public static double targetAngle = 90;

    private double getAbsoluteAngle() {
        return robot.drivetrain.getSensorIMU().getRobotOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosBot(hardwareMap);
        gyroController = new GyroController(2.03,0,0,0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // resetando o angulo
        robot.drivetrain.getSensorIMU().resetYaw();

        waitForStart();
        while(opModeIsActive()) {
            // definindo os coeficientes do dashboard
            gyroController.setPID(GYRO_COEFFICIENTS.p, GYRO_COEFFICIENTS.i, GYRO_COEFFICIENTS.d);
            gyroController.filterGain = GYRO_COEFFICIENTS.f;

            // giro de 90 graus por padrão
            double robotAngle = robot.drivetrain.getSensorIMU().getRobotOrientation(
                    AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS
            ).firstAngle;

            double output = gyroController.calculate(Math.toRadians(targetAngle), robotAngle);
            double motorOutput = Range.clip(output, -1.0,1.0);

            robot.drivetrain.setMotorsPower(-motorOutput, -motorOutput, motorOutput, motorOutput);

            telemetry.addData("target angle", targetAngle);
            telemetry.addData("robot angle", Math.toDegrees(robotAngle) );
            telemetry.addData("pid", output);
            telemetry.update();
        }
    }
}
