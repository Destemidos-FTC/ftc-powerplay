package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;

@TeleOp(name = "Teste ArmSystem", group = "Test")
public class TesteArmSystem extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmSystem armSystem = new ArmSystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            armSystem.moveArmsManually(gamepad2.left_stick_y);
            armSystem.moveForearmManually(gamepad2.right_stick_y);

            telemetry.addData("armA - position", armSystem.armA.getCurrentPosition());
            telemetry.addData("armB - position", armSystem.armB.getCurrentPosition());
            telemetry.addData("armC - position", armSystem.armC.getCurrentPosition());
            telemetry.addData("armD - position", armSystem.armD.getCurrentPosition());
            telemetry.update();
        }
    }
}
