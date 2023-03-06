package org.firstinspires.ftc.teamcode.math.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class MoveFourbar extends CommandBase {
    private Intake intake1;
    private double time;
    private double Power;

    public MoveFourbar(Intake intake, double servoPower, double timeout) {
        intake1 = intake;
        time = timeout;
    }

    @Override
    public void execute() {
        intake1.setMaxTimeout(time);
        intake1.moveFourbar(Power);
    }

    @Override
    public boolean isFinished() {
        return intake1.isFourbarStopped();
    }
}
