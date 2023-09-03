import problem.ArmConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static java.lang.Math.max;

public class PRM {

    private static final int NUM_SAMPLES = 2000; // 采样点数
    private static final double NEIGHBOR_RADIUS = 0.1; // 邻居半径
    private static final int NUM_NEIGHBORS = 5; // 邻居点数
    private static final int MAX_ITERATIONS = 1000; // 最大迭代次数

    private List<ArmConfig> samples;
    private PRMGraph roadmap;
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
        roadmap = new PRMGraph();
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
        samples.add(init);
        Random rand = new Random();
        int iteration = 0;
        while (iteration < NUM_SAMPLES) {
            ArmConfig randomConfig = getRandomSampleConfiguration();
            // 检查采样点是否有碰撞或者自出界
            boolean noValid = tester.hasCollision(randomConfig, this.obstacles)
                    || tester.hasSelfCollision(randomConfig)
                    || !tester.fitsBounds(randomConfig);
            if (!noValid) {
                samples.add(randomConfig);
                iteration++;
            }
        }
        samples.add(goal);
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
        System.out.println("开始建立边！");
        for(ArmConfig config : samples){
            List<ArmConfig> connected = new ArrayList<>();
            List<ArmConfig> neighbors = findNearestNeighbors(config);
            System.out.println(config+"有"+neighbors.size()+"个邻居！");
            for(ArmConfig neighbor : neighbors){
                if (isValidConnection(neighbor, config)) {
                    System.out.println(neighbor + "和"+ config +"可连接！");
                    connected.add(neighbor);
                }
            }
            roadmap.addConfig(config,connected);
        }
    }

    /**
     * 寻找距离给定配置最近的邻居。
     *
     * @param config 要查找邻居的配置
     * @return 最近的邻居列表
     */
    private List<ArmConfig> findNearestNeighbors(ArmConfig config) {
        System.out.println("寻找邻居！");
        List<ArmConfig> neighbors = new ArrayList<>();
        for (ArmConfig sample : samples) {
            double distance = calculateConfigDistance(config, sample);
//            if (distance <= NEIGHBOR_RADIUS && !config.equals(sample)) {
            if (!config.equals(sample)) {
                neighbors.add(sample);
            }
        }
        // 根据距离从近到远对邻居进行排序
        neighbors.sort((c1, c2) -> Double.compare(calculateConfigDistance(config, c1), calculateConfigDistance(config, c2)));
        return neighbors.subList(0, Math.min(NUM_NEIGHBORS, neighbors.size()));
    }

    /**
     * 计算两个配置之间的距离。距离由关节角度、夹持器中段长度和底座坐标之间的最大变化时间组成。
     *
     * @param config1 第一个配置
     * @param config2 第二个配置
     * @return 两个配置之间的距离，以最大变化时间为度量
     */
    private double calculateConfigDistance(ArmConfig config1, ArmConfig config2) {
        double jointAngleDifference = 0.0;
        double gripperLengthDifference = 0.0;
        double baseCoordinateDifference;
        // 计算关节角度之间的差异
        List<Double> jointAngles1 = config1.getJointAngles();
        List<Double> jointAngles2 = config2.getJointAngles();
        if (jointAngles1.size() == jointAngles2.size()) {
            for (int i = 0; i < jointAngles1.size(); i++) {
                double angle1 = jointAngles1.get(i);
                double angle2 = jointAngles2.get(i);
                // 使用角度之间的差异来计算距离
                jointAngleDifference = Math.max(Math.abs(angle1 - angle2),jointAngleDifference);
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
                    gripperLengthDifference = Math.max(Math.abs(length1 - length2),gripperLengthDifference);
                }
            }
        }
        // 计算底座坐标之间的差异
        Point2D base1 = config1.getBaseCenter();
        Point2D base2 = config2.getBaseCenter();
//        baseCoordinateDifference = base1.distance(base2);
        baseCoordinateDifference = Math.abs(base1.getX() - base2.getX()) + Math.abs(base1.getY() - base2.getY());
        return Math.max( jointAngleDifference/0.001 , Math.max(gripperLengthDifference/0.1 , baseCoordinateDifference/0.001));
    }

    /**
     * 检查是否可以连接两个ArmConfig，以确保路径不会转到障碍物或无法实现。
     *
     * @param config1
     * @param config2
     * @return 是否可以连接两个ArmConfig
     */
    private boolean isValidConnection(ArmConfig config1, ArmConfig config2) {
        // 获取ArmConfig之间的路径
        List<ArmConfig> path = generatePath(config1, config2);
        // 检查路径上的每个配置是否与障碍物相交
        for (ArmConfig pathConfig : path) {
            for (Obstacle obstacle : this.obstacles) {
                if (!tester.hasCollision(pathConfig, obstacle)) {
                    System.out.println("不可连接！");
                    return false; // 如果路径上的任何配置与障碍物相交，返回false
                }
            }
        }
        System.out.println("可以连接！");
        return  true;
    }
    /**
     * 生成两个ArmConfig之间的路径。
     *
     * @param config1 起始ArmConfig
     * @param config2 目标ArmConfig
     * @return 生成的路径，包括起始和目标配置
     */
    private List<ArmConfig> generatePath(ArmConfig config1, ArmConfig config2) {
        List<ArmConfig> path = new ArrayList<>();

        if (config1.getJointAngles().size() != config2.getJointAngles().size()) {
            throw new IllegalArgumentException("Configurations have different joint counts.");
        }

        // 获取起始和目标配置的信息
        Point2D base1 = config1.getBaseCenter();
        Point2D base2 = config2.getBaseCenter();
        List<Double> jointAngles1 = config1.getJointAngles();
        List<Double> jointAngles2 = config2.getJointAngles();
        List<Double> gripperLengths1 = config1.getGripperLengths();
        List<Double> gripperLengths2 = config2.getGripperLengths();

        // 计算底座平移的增量
        double baseIncrementX = (base2.getX() - base1.getX());
        double baseIncrementY = (base2.getY() - base1.getY());

        // 计算关节角度的最大变化
        double maxJointAngleChange = 0.0;
        for (int i = 0; i < jointAngles1.size(); i++) {
            double angle1 = jointAngles1.get(i);
            double angle2 = jointAngles2.get(i);
            double angleChange = Math.abs(angle2 - angle1);
            if (angleChange > maxJointAngleChange) {
                maxJointAngleChange = angleChange;
            }
        }
        // 计算夹持器长度的最大变化
        double maxGripperLengthChange = 0.0;
        for (int i = 0; i < gripperLengths1.size(); i++) {
            double length1 = gripperLengths1.get(i);
            double length2 = gripperLengths2.get(i);
            double lengthChange = Math.abs(length2 - length1);
            if (lengthChange > maxGripperLengthChange) {
                maxGripperLengthChange = lengthChange;
            }
        }
        // 根据三者中的最大变化计算插值步数
        int numSteps = (int) Math.ceil(Math.max(Math.max(Math.abs(baseIncrementX),
                Math.abs(baseIncrementY)), Math.max(maxJointAngleChange, maxGripperLengthChange)) );

        if (numSteps < 2) {
            numSteps = 2; // 至少需要两个步骤
        }
        // 计算每个基本步骤的增量
        baseIncrementX /= (numSteps - 1);
        baseIncrementY /= (numSteps - 1);

        for (int step = 0; step < numSteps; step++) {
            double baseX = base1.getX() + step * baseIncrementX;
            double baseY = base1.getY() + step * baseIncrementY;
            // 计算关节角度的插值
            List<Double> interpolatedJointAngles = new ArrayList<>();
            for (int i = 0; i < jointAngles1.size(); i++) {
                double angle1 = jointAngles1.get(i);
                double angle2 = jointAngles2.get(i);
                double interpolatedAngle = angle1 + (angle2 - angle1) * step / (numSteps - 1);
                interpolatedJointAngles.add(interpolatedAngle);
            }
            // 计算夹持器长度的插值
            List<Double> interpolatedGripperLengths = new ArrayList<>();
            for (int i = 0; i < gripperLengths1.size(); i++) {
                double length1 = gripperLengths1.get(i);
                double length2 = gripperLengths2.get(i);
                double interpolatedLength = length1 + (length2 - length1) * step / (numSteps - 1);
                interpolatedGripperLengths.add(interpolatedLength);
            }
            // 创建插值配置
            ArmConfig interpolatedConfig = new ArmConfig(new Point2D.Double(baseX, baseY), interpolatedJointAngles, interpolatedGripperLengths);
            path.add(interpolatedConfig);
        }

        return path;
    }



    public List<ArmConfig> getSamples() {
        return samples;
    }
    public PRMGraph getRoadmap() {
        return roadmap;
    }


}
