package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

@TeleOp(name = "TestServo", group = "Test")
public class TestServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        final Gripper gripperSys = new Gripper(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            if(isStopRequested()) return;

            RobotLogger.debugServoExInfo(telemetry, gripperSys.gripper);

            if(gamepad1.x) {
                gripperSys.closeGrip();
            }

            if (gamepad1.y) {
                gripperSys.releaseGrip();
            }

            if(gamepad1.right_bumper) {
                gripperSys.rotateGripper();
            }

            if (gamepad1.left_bumper) {
                gripperSys.returnToCollectPostion();
            }
        }
    }



}