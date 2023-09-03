import problem.ArmConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

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
    // 建立道路图
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
    //
    private double calculateConfigDistance(ArmConfig config1, ArmConfig config2) {
        // TODO: 计算两个配置之间的距离
        // 这通常涉及到关节角度之间的差异和夹持器中段长度之间的差异

        return 0.0;
    }

    public List<ArmConfig> getRoadmap() {
        return roadmap;
    }
}
