import problem.ArmConfig;

import java.util.Random;
public class GerRandomString {
    private static final double MIN_X = 0.0;
    private static final double MAX_X = 1.0;
    private static final double MIN_Y = 0.0;
    private static final double MAX_Y = 1.0;
    private static final double MIN_JOINT_ANGLE = -150.0;
    private static final double MAX_JOINT_ANGLE = 150.0;
    private static final double MIN_GRIPPER_LENGTH = 0.03;
    private static final double MAX_GRIPPER_LENGTH = 0.07;

    /**
     * Generates a random configuration string based on the specified parameters.
     * 根据指定参数生成随机配置字符串。
     *
     * @param withGripper True if the configuration should include gripper information, false otherwise.
     * @param jointCount The number of joint angles to generate.
     * @return A formatted string representing the random configuration.
     */
    public static String gerRandomString(boolean withGripper, int jointCount) {
        Random rand = new Random();

        // 生成底座坐标
        double baseX = MIN_X + rand.nextDouble() * (MAX_X - MIN_X);
        double baseY = MIN_Y + rand.nextDouble() * (MAX_Y - MIN_Y);

        StringBuilder configStr = new StringBuilder();
        // 底座中心坐标
        configStr.append(String.format("%.2f %.2f ", baseX, baseY));

        // 生成关节角度
        for (int i = 0; i < jointCount; i++) {
            double jointAngle = MIN_JOINT_ANGLE + rand.nextDouble() * (MAX_JOINT_ANGLE - MIN_JOINT_ANGLE);
            configStr.append(String.format("%.2f ", jointAngle));
        }

        // 如果需要生成带夹持器的配置，生成夹持器中段长度
        if (withGripper) {
            for (int i = 0; i < 4; i++) {
                double gripperLength = MIN_GRIPPER_LENGTH + rand.nextDouble() * (MAX_GRIPPER_LENGTH - MIN_GRIPPER_LENGTH);
                configStr.append(String.format("%.2f ", gripperLength));
            }
        }

        return configStr.toString().trim();
    }
}
