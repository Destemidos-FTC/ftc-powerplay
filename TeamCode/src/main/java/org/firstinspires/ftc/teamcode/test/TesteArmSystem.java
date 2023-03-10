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
            arms.moveArmsManually(gamepad1.left_stick_y);
            forearm.moveForearmManually(gamepad1.right_stick_y);

            telemetry.addData("arm - position", arms.armA.getCurrentPosition());
            telemetry.addData("forearm - position", forearm.forearmMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
