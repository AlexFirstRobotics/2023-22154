package org.firstinspires.ftc.teamcode.armsubsystem;

public final class ArmConstants {

    public static class ARM_POSITIONS {
        public static class PIVOT {
            public static int HOME_POSITION = 30;
            public static int SCORE_POSITION = 830;
            public static int SCORE_EXTRA_POSITION = -900;
            public static int PICKUP_POSITION = 194; //-100
            public static int CLIMB_UP_POSITION = 1150;
            public static int CLIMB_DOWN_POSITION = 100;
        }
        public static class EXTEND {
            public static int HOME_POSITION = 0;
            public static int SCORE_POSITION = -1743;
            public static int PICKUP_POSITION = -524; //-785
        }
        public static class GRAB {
            public static double OPEN = 0.40;
            public static double CLOSE = 0.55;
        }
    }

}