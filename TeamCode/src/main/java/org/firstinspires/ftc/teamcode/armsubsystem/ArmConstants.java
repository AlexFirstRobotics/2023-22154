package org.firstinspires.ftc.teamcode.armsubsystem;

public final class ArmConstants {

    public static class ARM_POSITIONS {
        public static class PIVOT {
            public static int HOME_POSITION = 20;
            public static int SCORE_POSITION = -820;
            public static int SCORE_EXTRA_POSITION = -900;
            public static int PICKUP_POSITION = -130; //-100
            public static int CLIMB_UP_POSITION = -1100;
            public static int CLIMB_DOWN_POSITION = 100;
        }
        public static class EXTEND {
            public static int HOME_POSITION = 0;
            public static int SCORE_POSITION = -2000;
            public static int PICKUP_POSITION = -550; //-785
        }
        public static class GRAB {
            public static double OPEN = 0.40;
            public static double CLOSE = 0.55;
        }
    }

}