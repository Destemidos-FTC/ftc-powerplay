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
            armSystem.moveArms(gamepad2);

            if(gamepad2.x) {
                armSystem.moveToAngleWithPID(90.0);
            }
        }
    }

}
