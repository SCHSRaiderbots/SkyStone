package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.SCHSConstants.*;

public class Raiderbot {

    private SCHSDrive robotChassis;

    public void initialize(HardwareMap hardwareMap) {

        robotChassis = new SCHSDrive();
        robotChassis.initialize(hardwareMap);

    }

    public void cleanShutDown() {

    }

    public void testFunctions() {
        // test drive and turn functions

        robotChassis.moveStraightWithGyro(POWER_FULL_FORWARD, 24);
        robotChassis.turnWithGyro(POWER_TURN_SPEED, 45, LEFT_TURN);
        //robotChassis.moveStraightWithGyro(POWER_FULL_FORWARD, 12);
        //robotChassis.moveStraightWithGyro(POWER_FULL_BACKWARD, -24);
        //robotChassis.turnWithGyro(POWER_TURN_SPEED, 180, RIGHT_TURN);

    }

}
