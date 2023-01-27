package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

@Autonomous(name = "Test Giroscopio", group = "Test")
public class TestGiroscopio extends LinearOpMode {
    private DestemidosBot robot;
    private PIDController pidController;
    private double lastAngles;
    private double currAngle;
    public static int STAGE = 1;

    private double getAbsoluteAngle() {
        return robot.drivetrain.getSensorIMU().getRobotOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    private void resetAngle() {
        lastAngles = getAbsoluteAngle();
        currAngle = 0;
    }

    private double correctAngle(double angle) {
        if (angle < -180) {
            angle += 360;
        } else if (angle > 180) {
            angle -= 360;
        }

        return angle;
    }

    private double getAngle() {
        double actualAngle = getAbsoluteAngle();
        double deltaAngle = actualAngle - lastAngles;

        deltaAngle = correctAngle(deltaAngle);

        double currAngle = deltaAngle;
        lastAngles = actualAngle;
        telemetry.addData("Test Giro", actualAngle);
        return currAngle;
    }

    private void turnRobot(double degrees, double motorPower){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            if(error < 0) {
                motorPower = motorPower * -1;
            }
            robot.drivetrain.setAllPower(motorPower);

            error = degrees - getAngle();
            telemetry.addData("error", error);
        }

        robot.drivetrain.setAllPower(0);
    }

    private void turnTo(double degrees){
        double error = degrees - getAbsoluteAngle();
        error = correctAngle(error);

        turnRobot(error, 0.3);
    }

    private void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    private void turnToPID(double targetAngle) {

        // (ramalho): o código original pede o 'ultimo slope" medido
        // slope é a taxa de variação, então deve ser uma substituição equivalente
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pidController.getD() > 0.75) {
            double motorPower = pidController.calculate(getAbsoluteAngle());
            robot.drivetrain.setAllPower(motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pidController.getD());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.drivetrain.setAllPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosBot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (STAGE == 1) {
            turnRobot(90, 0.3);
            sleep(3000);
            turnTo(-90);
        } else if (STAGE == 2) {
            turnPID(90);
        }

    }
}
