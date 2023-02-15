package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.GripSystem;

@Disabled
@TeleOp(name = "TestServo", group = "Test")
public class TestServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final DestemidosBot robot = new DestemidosBot(hardwareMap);
        final ServoControllerEx servoController = (ServoControllerEx) robot.servoMão.getController();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            if(isStopRequested()) return;

            ArmSystem.movimentarBraço(gamepad2, robot);

            GripSystem.coletarCones(gamepad2, robot);

            debugServoInfo(robot.servoMão, telemetry);
            telemetry.addData("Servo PWM Range: ",
                    servoController.getServoPwmRange(robot.servoMão.getPortNumber())
            );
            telemetry.addData("Servo Connection Info: ",
                    servoController.getConnectionInfo()
            );
        }
    }


    void debugServoInfo(Servo servo, Telemetry telemetry) {
        telemetry.addData("Servo Name:", servo.getDeviceName());
        telemetry.addData("Servo Position:", servo.getPosition());
        telemetry.addData("Servo Direction:", servo.getDirection());
        telemetry.addData("Servo Port:", servo.getPortNumber());
    }
}