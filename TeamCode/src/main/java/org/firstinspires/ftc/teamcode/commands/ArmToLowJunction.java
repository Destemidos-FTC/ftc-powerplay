package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

public class ArmToLowJunction extends SequentialCommandGroup {

    public ArmToLowJunction(DestemidosBot robot) {
        super(
                //new InstantCommand(()->robot.armSystem.setForearmPosition(ArmSystem.ForearmStage.LOW)),
                //new InstantCommand(()->robot.armSystem.setArmPosition(ArmSystem.ArmStage.LOW))
                new InstantCommand(()->robot.simpleArm.goToPosition(ArmSystem.ArmStage.LOW))

        );
    }
}
