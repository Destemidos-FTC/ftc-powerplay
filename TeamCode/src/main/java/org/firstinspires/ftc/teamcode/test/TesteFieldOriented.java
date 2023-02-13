package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

@TeleOp(name = "Teste - Field Oriented", group = "Test")
public class TesteFieldOriented extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        // setup the hardware components
        DestemidosBot robot = new DestemidosBot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {

            robot.drivetrain.driveFieldOriented(gamepad1);

            telemetry.update();
        }
    }
}
