package frc.robot;

public class Constants {
    // arm constants
    public static final double armKP = 7.5;
    public static final double armKI = 0.015;
    public static final double armKD = 55;

    public static final double maxArmSpeed = .7;   
    public static final int maxArmError = 5;

    // wrist constants
    public static final double wristKP = 55;
    public static final double wristKI = 0.001;
    public static final double wristKD = 90;

    public static final double maxWristSpeed = 0.60;   
    public static final int maxWristError = 3;

    public static final int kTimeoutMs = 30;
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;

    public static final int armOffset = 522;
    public static final int wristOffset = 299;
}