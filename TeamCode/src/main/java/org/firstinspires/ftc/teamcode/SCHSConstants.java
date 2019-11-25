package org.firstinspires.ftc.teamcode;

public class SCHSConstants {

    // distance measurements for each movement in inches

    // angle measurements for each turn

    //left or right direction for each turn - int where 1 = left and 2 = right
    static final int LEFT_TURN = 1;
    static final int RIGHT_TURN = 2;

    //servo constants - direction, increment
    static final int SERVO_DIRECTION_LEFT = 1;
    static final int SERVO_DIRECTION_RIGHT = 2;
    static final double INCREMENT   = 0.01;
    static final int    CYCLE_MS    =   50;

    // color values as that the color sensor would detect from each picture - may be changed to red/green/blue/alpha values

    // power constants between 1 and -1
    static final double POWER_FULL_FORWARD = 1;
    static final double POWER_FULL_BACKWARD = -1;
    static final double POWER_HALF_FORWARD = 0.5;
    static final double POWER_HALF_BACKWARD = -0.5;
    static final double POWER_TURN_SPEED = 0.25;

    //Tensor Flow Object detection
    static final String VUFORIA_KEY = "AUnX7nP/////AAABmZjfOTd2skx4p/r+LBA29VQAFar5mbPnEfGtcl78mMIqK+EtsUOR33zwyiDCmj1oYMUx0P4eWZGi6EMhZgTM66/5llx5azKwGGxGmTJUGotbAekyZgxYR7SWDme6xMYGR68jZcR9rkvJxfB1ZKFytPXWeRpwzSAQJ0VACF/hdguUyfA6SSkF2dnc/iH76TkSV3hA4zz0v3wjHfQmmNBvrtgPklvfOTX2f+G5tBfBq75PEx52LaX+tOPTtBajR9MFwVT26kcqFz2GJCEBgjO3PX1St0xNJBqbbudKvZ+B/6xWuVhwHVqwOgy/RsuHLBFskh4n9Ec1xnuB9uCnQXrrliEtcR1TbnmIEYTX6FZtxF5H";
    //static final int FOCAL_LENGTH = 27; //mm
    //static final int REAL_HEIGHT_GOLD = 50; //mm
    //static final int IMG_HEIGHT = 2448; //pixels
    //static final int CAMERA_HEIGHT = 264; //mm, previous value 35
    //static final int SCAN_BALLS_TIME = 5000; // milliseconds

    //Vuforia Picture Detection
    //static final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
    //static final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    //static final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
    //static final int SCAN_PICTURE_TIME = 3000; // milliseconds

    //core hex motor constants
    static final double CHMOTOR_COUNTS_PER_REVOLUTION = 288;
    static final double REAR_WHEEL_BASE_= 12; //inches
    static final double TRACTION_WHEEL_DIAMETER = 90 * 0.0393701; //90mm converted to inches

    //formula for inches to counts (encoder value): 288 counts/revolution
    //1 revolution = 90pi mm = 3.54331pi inches
    //total counts = 288*rev
    //x inches / 3.54331pi = # rev
    //encoder value = (288*x)/(3.54331pi) = 310.466 at x = 12 inches

    //formula for degrees (encoder value): (a/360)*(3.54331pi) = y inches
    //encoder value = (288*y)/(12pi) = 310.466 at y inches for a degrees
    //at a = 90 degrees, encoder value = 243.839

}
