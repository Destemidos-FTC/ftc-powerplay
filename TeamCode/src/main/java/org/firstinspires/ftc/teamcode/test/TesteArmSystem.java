package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.ForearmSystem;

@TeleOp(name = "Teste ArmSystem", group = "Test")
public class TesteArmSystem extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmSystem arms = new ArmSystem(hardwareMap);
        ForearmSystem forearm = new ForearmSystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            arms.moveArmsManually(gamepad2.left_stick_y);
            forearm.moveForearmManually(gamepad2.right_stick_y);

            telemetry.addData("armA - position", arms.armA.getCurrentPosition());
            telemetry.addData("armB - position", arms.armB.getCurrentPosition());
            telemetry.addData("armC - position", forearm.armC.getCurrentPosition());
            telemetry.addData("armD - position", forearm.armD.getCurrentPosition());
            telemetry.update();
        }
    }
}
