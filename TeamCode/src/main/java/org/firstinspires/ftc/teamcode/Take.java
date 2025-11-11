package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Take {

    private final DcMotor intake;
    private boolean TToggle = false;

    public Take(DcMotor intake) {
        this.intake = intake;
    }

    public void execute() {
        if (!TToggle) {
            intake.setPower(1);
            TToggle = true;
        } else {
            intake.setPower(0);
            TToggle = false;
        }
    }
}
