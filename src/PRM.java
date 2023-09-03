import problem.ArmConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class PRM {

    private static final int NUM_SAMPLES = 1000; // 采样点数
    private static final double NEIGHBOR_RADIUS = 0.1; // 邻居半径
    private static final int NUM_NEIGHBORS = 5; // 邻居点数
    private static final int MAX_ITERATIONS = 1000; // 最大迭代次数

    private List<ArmConfig> samples;
    private List<ArmConfig> roadmap;
    private ProblemSpec problemSpec;
    private List<Obstacle> obstacles;
    private ArmConfig init;
    private ArmConfig goal;
    private Tester tester = new Tester();
    private boolean hasGripper;

    /**
     *Initializes a PRM (Probabilistic Roadmap) planner with the given problem specification.
     *
     * @param problemSpec problemSpec The problem specification.
     */
    public PRM(ProblemSpec problemSpec) {
        samples = new ArrayList<>();
        roadmap = new ArrayList<>();
        this.problemSpec = problemSpec;
        this.obstacles = problemSpec.getObstacles();
        this.init = problemSpec.getInitialState();
        this.goal = problemSpec.getGoalState();
        this.hasGripper = init.hasGripper();
    }

    /**
     * Generates the roadmap.
     */
    public void generateRoadmap() {
        // 1. Generate random sample points
        // 1. 生成随机采样点
        generateSamples();
        // 2. Build the roadmap
        // 2. 建立道路图
        buildRoadmap();
    }

    /**
     * Generates random samples and adds them to the sample list.
     * 生成随机样本并将其添加到样本列表中。
     */
    private void generateSamples() {
        Random rand = new Random();
        int iteration = 0;
        while (iteration < NUM_SAMPLES) {
            ArmConfig randomConfig = getRandomSampleConfiguration();
            // 检查采样点是否有碰撞或者自出界
            boolean validSample = tester.hasCollision(randomConfig, this.obstacles)
                    && tester.hasSelfCollision(randomConfig)
                    && tester.fitsBounds(randomConfig);
            if (validSample) {
                samples.add(randomConfig);
                iteration++;
            }
        }
    }
    /**
     * Generates a random ArmConfig for sampling.
     * 生成一个随机 ArmConfig 供采样。
     * @return A random ArmConfig instance.
     */
    private ArmConfig getRandomSampleConfiguration() {
        Random rand = new Random();
        String configStr = GerRandomString.gerRandomString(this.hasGripper, problemSpec.getJointCount());
        if (hasGripper){
            return new ArmConfig(configStr);
        }
        else {
            return new ArmConfig(configStr,hasGripper);
        }
    }
    /**
     *
     */
    private void buildRoadmap() {
        roadmap.add(this.init);
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            ArmConfig randomConfig = getRandomSampleConfiguration();
            List<ArmConfig> nearestNeighbors = findNearestNeighbors(randomConfig);
            for (ArmConfig neighbor : nearestNeighbors) {
                tester.isValidStep(randomConfig,neighbor);
                    roadmap.add(randomConfig);
                    roadmap.add(neighbor);
            }
        }
        roadmap.add(this.goal);
    }
    // 获得随机配置

    // 查找一个配置的邻配置
    private List<ArmConfig> findNearestNeighbors(ArmConfig config) {
        List<ArmConfig> neighbors = new ArrayList<>();
        for (ArmConfig sample : samples) {
            double distance = calculateConfigDistance(config, sample);
            if (distance <= NEIGHBOR_RADIUS && !config.equals(sample)) {
                neighbors.add(sample);
            }
        }
        neighbors.sort((c1, c2) -> Double.compare(calculateConfigDistance(config, c1), calculateConfigDistance(config, c2)));
        return neighbors.subList(0, Math.min(NUM_NEIGHBORS, neighbors.size()));
    }

    /**
     * 计算两个配置之间的距离。
     *
     * @param config1
     * @param config2
     * @return
     */
    private double calculateConfigDistance(ArmConfig config1, ArmConfig config2) {
        double jointAngleDifference = 0.0;
        double gripperLengthDifference = 0.0;
        double baseCoordinateDifference = 0.0;
        // 计算关节角度之间的差异
        List<Double> jointAngles1 = config1.getJointAngles();
        List<Double> jointAngles2 = config2.getJointAngles();
        if (jointAngles1.size() == jointAngles2.size()) {
            for (int i = 0; i < jointAngles1.size(); i++) {
                double angle1 = jointAngles1.get(i);
                double angle2 = jointAngles2.get(i);
                // 使用角度之间的差异来计算距离
                jointAngleDifference += Math.abs(angle1 - angle2);
            }
        }
        // 如果配置包含夹持器
        if (config1.hasGripper() && config2.hasGripper()) {
            List<Double> gripperLengths1 = config1.getGripperLengths();
            List<Double> gripperLengths2 = config2.getGripperLengths();
            // 确保夹持器长度列表长度相同
            if (gripperLengths1.size() == gripperLengths2.size()) {
                for (int i = 0; i < gripperLengths1.size(); i++) {
                    double length1 = gripperLengths1.get(i);
                    double length2 = gripperLengths2.get(i);
                    // 使用夹持器中段长度之间的差异来计算距离
                    gripperLengthDifference += Math.abs(length1 - length2);
                }
            }
        }
        // 计算底座坐标之间的差异
        Point2D base1 = config1.getBaseCenter();
        Point2D base2 = config2.getBaseCenter();
        baseCoordinateDifference = base1.distance(base2);
        // 这里可以根据需要自定义距离计算方法，例如加权不同部分的差异
        return jointAngleDifference + gripperLengthDifference + baseCoordinateDifference;
    }

    /**
     * 检查是否可以连接两个ArmConfig，以确保路径不会转到障碍物或无法实现。
     *
     * @param config1
     * @param config2
     * @return 是否可以连接两个ArmConfig
     */
    private boolean isValidConnection(ArmConfig config1, ArmConfig config2, Obstacle obstacles) {
        // TODO: 实现检查是否可以连接的逻辑，以确保路径不会转到障碍物或无法实现。
        return true;
    }
    public List<ArmConfig> getRoadmap() {
        return roadmap;
    }

    private List<ArmConfig> generatePath(ArmConfig config1, ArmConfig config2, int numSteps) {
        List<ArmConfig> path = new ArrayList<>();

        if (config1.getJointAngles().size() != config2.getJointAngles().size()) {
            throw new IllegalArgumentException("Configurations have different joint counts.");
        }

        if (numSteps < 2) {
            throw new IllegalArgumentException("Number of steps must be at least 2 for linear interpolation.");
        }

        for (int step = 0; step < numSteps; step++) {
            double ratio = (double) step / (double) (numSteps - 1);
            List<Double> interpolatedJointAngles = new ArrayList<>();

            for (int i = 0; i < config1.getJointAngles().size(); i++) {
                double jointAngle1 = config1.getJointAngles().get(i);
                double jointAngle2 = config2.getJointAngles().get(i);
                double interpolatedAngle = jointAngle1 + ratio * (jointAngle2 - jointAngle1);
                interpolatedJointAngles.add(interpolatedAngle);
            }

            ArmConfig interpolatedConfig = new ArmConfig(
                    config1.getBaseCenter(),
                    interpolatedJointAngles,
                    config1.getLinks(),
                    config1.hasGripper(),
                    config1.getGripperLengths()
            );

            path.add(interpolatedConfig);
        }

        return path;
    }
}
