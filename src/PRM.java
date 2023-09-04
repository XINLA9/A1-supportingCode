import problem.ArmConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.util.*;

import static java.lang.Math.max;

public class PRM {

    private static final int NUM_SAMPLES = 10000; // 采样点数
    private static final double NEIGHBOR_RADIUS = 0.1; // 邻居半径
    private static final int NUM_NEIGHBORS = 50; // 邻居点数
    private static final int MAX_ITERATIONS = 1000; // 最大迭代次数

    private List<ArmConfig> samples;
    private static PRMGraph roadmap;
    private ProblemSpec problemSpec;
    private List<Obstacle> obstacles;
    private ArmConfig init;
    private static ArmConfig goal;
    private Tester tester = new Tester();
    private static boolean hasGripper;

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
        samples.add(goal);
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
    }
    /**
     * Generates a random ArmConfig for sampling.
     * 生成一个随机 ArmConfig 供采样。
     * @return A random ArmConfig instance.
     */
    private ArmConfig getRandomSampleConfiguration() {
        String configStr = GerRandomString.gerRandomString(hasGripper, problemSpec.getJointCount());
        if (hasGripper){
            return new ArmConfig(configStr,hasGripper);
        }
        else {
            return new ArmConfig(configStr);
        }
    }
    public static class GerRandomString {
        private static final double MIN_X = 0.04;
        private static final double MAX_X = 0.96;
        private static final double MIN_Y = 0.04;
        private static final double MAX_Y = 0.96;
        private static final double MIN_JOINT_ANGLE = -5 * Math.PI / 6.0;
        private static final double MAX_JOINT_ANGLE = 5 * Math.PI / 6.0;
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
    /**
     *
     */
    private void buildRoadmap() {
        for(ArmConfig config : samples) {
            List<ArmConfig> neighbors = findNearestNeighbors(config, NUM_NEIGHBORS);
            List<ArmConfig> connected = new ArrayList<>();

            for(ArmConfig neighbor : neighbors) {
                if (isValidConnection(config, neighbor)) {
                    connected.add(neighbor);
                }
            }
            roadmap.addConfig(config, connected);
        }
        roadmap.setStart(init);
        roadmap.setGoal(goal);
    }

    /**
     * 寻找距离给定配置最近的邻居，最多添加指定数量的邻居。
     *
     * @param config 要查找邻居的配置
     * @param maxNeighbors 最大邻居数
     * @return 最多最近的邻居列表，最多包含maxNeighbors个邻居
     */
    private List<ArmConfig> findNearestNeighbors(ArmConfig config, int maxNeighbors) {
        int num = 0;
        List<ArmConfig> neighbors = new ArrayList<>();
        for (ArmConfig sample : samples) {
            double distance = calculateConfigDistance(config, sample);
            if (distance <= NEIGHBOR_RADIUS && !config.equals(sample)) {
                if (isValidConnection(config, sample)) { // 检查是否可连接
                    neighbors.add(sample);
                    num++;
                }
            }
            if (num >=maxNeighbors){
                break;
            }
        }
        // 根据距离从近到远对邻居进行排序
        neighbors.sort((c1, c2) -> Double.compare(calculateConfigDistance(config, c1), calculateConfigDistance(config, c2)));
        return neighbors;
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
                if (tester.hasCollision(pathConfig, obstacle)) {
                    return false; // 如果路径上的任何配置与障碍物相交，返回false
                }
            }
        }
//        System.out.println(config1+"和"+config2+"可以连接！");
        return  true;
    }
    /**
     * 计算两个配置之间的距离。距离由关节角度、夹持器中段长度和底座坐标之间的最大变化时间组成。
     *
     * @param config1 第一个配置
     * @param config2 第二个配置
     * @return 两个配置之间的距离，以最大变化时间为度量
     */
    private double calculateConfigDistance(ArmConfig config1, ArmConfig config2) {
        Point2D base1 = config1.getBaseCenter();
        Point2D base2 = config2.getBaseCenter();

        return base1.distance(base2);
    }


    /**
     * 生成两个ArmConfig之间的插值路径。
     *
     * @param config1 起始ArmConfig
     * @param config2 目标ArmConfig
     * @return 生成的路径，包括起始和目标配置
     */
    public static List<ArmConfig> generatePath(ArmConfig config1, ArmConfig config2) {
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

        double baseDifference = base1.distance(base2);
        List<Double> jointAngleDifferences = new ArrayList<>();
        for (int i = 0; i < jointAngles1.size(); i++) {
            double angle1 = jointAngles1.get(i);
            double angle2 = jointAngles2.get(i);
            double angleDiff = angle2 - angle1;
            jointAngleDifferences.add(angleDiff);
        }
        List<Double> gripperLengthDifference = new ArrayList<>();
        for (int i = 0; i < gripperLengths1.size(); i++) {
            double length1 = gripperLengths1.get(i);
            double length2 = gripperLengths2.get(i);
            double lengthDiff = length2 - length1;
            gripperLengthDifference.add(lengthDiff);
        }

        // 底座平移限制
        double baseChangeLimit = 0.001;
        // 计算关节角度变化限制
        double JointAngleChangeLimit = 0.10;
        double maxJointChange = Collections.max(jointAngleDifferences);

        // 计算夹持器长度的变化限制
        double gripperLengthChangeLimit = 0.001;
        double maxGripperChange = 0;
        if (hasGripper){
            maxGripperChange = Collections.max(gripperLengthDifference);
        }


        // 根据四者中的最大变化计算插值步数
        int numSteps = (int) Math.ceil(
                Math.max(Math.abs(baseDifference / baseChangeLimit),
                        Math.max(maxJointChange / JointAngleChangeLimit, maxGripperChange/gripperLengthChangeLimit)
        ));

        if (numSteps < 2) {
            numSteps = 2; // 至少需要两个步骤
        }

        // 计算每个基本步骤的增量
        double baseIncrementX = (base2.getX() - base1.getX()) /  (numSteps - 1);
        double baseIncrementY = (base2.getY() - base1.getY()) / (numSteps -1);

        List<Double> jointIncrement = new  ArrayList<>();
        for(Double difference : jointAngleDifferences){
            jointIncrement.add(difference / (numSteps - 1));
        }
        List<Double> gripperIncrement= new  ArrayList<>();
        for(Double difference : gripperLengthDifference){
            gripperIncrement.add(difference / (numSteps - 1));
        }


        for (int step = 0; step < numSteps; step++) {
            double baseX = base1.getX() + step * baseIncrementX;
            double baseY = base1.getY() + step * baseIncrementY;

            // 计算关节角度的插值
            List<Double> interpolatedJointAngles = new ArrayList<>();
            for (int i = 0; i < jointAngles1.size(); i++) {
                double angle = jointAngles1.get(i);
                double interpolatedAngle = angle + jointIncrement.get(i) * step;
                interpolatedJointAngles.add(interpolatedAngle);
            }

            // 计算夹持器长度的插值
            List<Double> interpolatedGripperLengths = new ArrayList<>();
            for (int i = 0; i < gripperLengths1.size(); i++) {
                double length1 = gripperLengths1.get(i);
                double length2 = gripperLengths2.get(i);
                double interpolatedLength = length1 + gripperIncrement.get(i) * step;
                interpolatedGripperLengths.add(interpolatedLength);
            }

            // 创建插值配置
            if (hasGripper) {
                ArmConfig interpolatedConfig = new ArmConfig(new Point2D.Double(baseX, baseY), interpolatedJointAngles, interpolatedGripperLengths);
                path.add(interpolatedConfig);
            } else {
                ArmConfig interpolatedConfig = new ArmConfig(new Point2D.Double(baseX, baseY), interpolatedJointAngles);
                path.add(interpolatedConfig);
            }
        }

        return path;
    }

    public List<ArmConfig> getSamples() {
        return samples;
    }
    public PRMGraph getRoadmap() {
        return roadmap;
    }

    public static class PRMGraph {
        private final Map<ArmConfig, List<ArmConfig>> graph;
        private ArmConfig start;
        private ArmConfig goal;
        public PRMGraph() {graph = new HashMap<>();}
        // 添加配置和其邻居
        public void addConfig(ArmConfig config, List<ArmConfig> neighbors) {
            graph.put(config, neighbors);
        }
        // 获取配置的邻居列表
        public List<ArmConfig> getNeighbors(ArmConfig config) {
            return graph.getOrDefault(config, new ArrayList<>());
        }
        // 检查两个配置之间是否有边
        public boolean hasEdge(ArmConfig config1, ArmConfig config2) {
            List<ArmConfig> neighbors = getNeighbors(config1);
            return neighbors.contains(config2);
        }
        public Map<ArmConfig, List<ArmConfig>> getGraph() {
            return graph;
        }
        // 添加边连接两个配置
        public void addEdge(ArmConfig config1, ArmConfig config2) {
            if (!hasEdge(config1, config2)) {
                List<ArmConfig> neighbors1 = getNeighbors(config1);
                neighbors1.add(config2);

                List<ArmConfig> neighbors2 = getNeighbors(config2);
                neighbors2.add(config1);
            }
        }
        public ArmConfig getStart() {
            return start;
        }
        public void setStart(ArmConfig start) {
            this.start = start;
        }
        public ArmConfig getGoal() {
            return goal;
        }
        public void setGoal(ArmConfig goal) {
            this.goal = goal;
        }
    }

    /**
     *
     */
    public static class PRMSearch {
        public static List<ArmConfig> search(PRM.PRMGraph graph) {
            // 使用优先级队列按照评估函数排序
            PriorityQueue<SearchNode> openQueue = new PriorityQueue<>(new Comparator<SearchNode>() {
                @Override
                public int compare(SearchNode node1, SearchNode node2) {
                    // 根据 cost + heuristic 进行比较
                    double f1 = node1.getCost() + node1.getHeuristic();
                    double f2 = node2.getCost() + node2.getHeuristic();
                    return Double.compare(f1, f2);
                }
            });

            Set<ArmConfig> closedList = new HashSet<>();
            Map<ArmConfig, SearchNode> nodeMap = new HashMap<>();
            Map<ArmConfig, List<ArmConfig>> neighbors = graph.getGraph();

            ArmConfig start = graph.getStart();
            ArmConfig goal = graph.getGoal();
            System.out.println(neighbors);
            // 初始化起始节点
            SearchNode startNode = new SearchNode(start, 0);
            openQueue.offer(startNode);
            nodeMap.put(start, startNode);
            SearchNode currentNode = null;

            while (!openQueue.isEmpty()) {
                currentNode = openQueue.poll();
//                System.out.println("弹出结点"+currentNode);
                ArmConfig currentConfig = currentNode.getConfig();

                if (currentConfig.equals(goal)) {
                    // 找到目标配置，回溯路径
                    return reconstructPath(currentNode);
                }

                closedList.add(currentConfig);

                for (ArmConfig neighbor : neighbors.get(currentConfig)) {
                    if (closedList.contains(neighbor)) {
                        continue; // 已经探索过的节点，跳过
                    }

                    double newCost = currentNode.getCost() + heuristic(currentConfig, neighbor);
                    SearchNode neighborNode = nodeMap.get(neighbor);

                    if (neighborNode == null || newCost < neighborNode.getCost()) {
                        double heuristic = heuristic(neighbor, goal);
                        double totalCost = newCost + heuristic;

                        if (neighborNode == null) {
                            neighborNode = new SearchNode(neighbor,totalCost, currentNode);
                            nodeMap.put(neighbor, neighborNode);
                        } else {
                            neighborNode.updateCost(newCost, heuristic, currentNode);
                        }

                        openQueue.offer(neighborNode);
                    }
                }
            }
            // 未找到路径
            System.out.println("Does not find a path!");
//            return null;
            return reconstructPath(currentNode);
        }


        /**
         * 从当前节点反向重构路径。
         *
         * @param node 最终节点
         * @return 从起始节点到最终节点的路径
         */
        private static List<ArmConfig> reconstructPath(SearchNode node) {
            List<ArmConfig> path = new ArrayList<>();
            path.add(node.getConfig());
            while (node.getParent() != null) {
                node = node.parent;
                path.add(node.getConfig());
            }
            Collections.reverse(path);
            return path;
        }

        /**
         * 计算两个ArmConfig对象之间的距离。
         *
         * @param armConfig1 第一个ArmConfig对象
         * @param armConfig2 第二个ArmConfig对象
         * @return 两个ArmConfig对象之间的距离
         */
        private static double cost(ArmConfig armConfig1, ArmConfig armConfig2){
            // Get the base position for both configurations
            Point2D base1 = armConfig1.getBaseCenter();
            Point2D base2 = armConfig2.getBaseCenter();
            // 获取两个配置的关节角度
            List<Double> jointAngles1 = armConfig1.getJointAngles();
            List<Double> jointAngles2 = armConfig2.getJointAngles();
            // 计算底座位置的欧几里得距离
            double baseDistance = base1.distance(base2);
            // 计算关节角度之间的差异
            double jointAngleDifference = 0.0;
            for (int i = 0; i < jointAngles1.size(); i++) {
                double angle1 = jointAngles1.get(i);
                double angle2 = jointAngles2.get(i);
                double angleDiff = Math.abs(angle2 - angle1);
                jointAngleDifference += angleDiff;
            }
            // 初始化夹持器长度差异
            double gripperLengthDifference = 0.0;
            if (armConfig1.hasGripper()) {
                // 如果有夹持器，计算夹持器长度之间的差异
                List<Double> gripperLengths1 = armConfig1.getGripperLengths();
                List<Double> gripperLengths2 = armConfig2.getGripperLengths();

                for (int i = 0; i < gripperLengths1.size(); i++) {
                    double length1 = gripperLengths1.get(i);
                    double length2 = gripperLengths2.get(i);
                    double lengthDiff = Math.abs(length2 - length1);
                    gripperLengthDifference += lengthDiff;
                }
            }
            // 将底座距离、关节角度差异和夹持器长度差异组合成总距离

            return 0.0;
        }
        private static double heuristic(ArmConfig config1, ArmConfig config2){
            Point2D base1 = config1.getBaseCenter();
            Point2D base2 = config2.getBaseCenter();
            List<Double> jointAngles1 = config1.getJointAngles();
            List<Double> jointAngles2 = config2.getJointAngles();
            List<Double> gripperLengths1 = config1.getGripperLengths();
            List<Double> gripperLengths2 = config2.getGripperLengths();
            // 计算底座平移的增量
            double baseChangeLimit = 0.001;
            double baseIncrementX = (base2.getX() - base1.getX());
            double baseIncrementY = (base2.getY() - base1.getY());
            // 计算关节角度的最大变化
            double JointAngleChangeLimit = 0.10;
            double maxJointAngleChange = 0;
            for (int i = 0; i < jointAngles1.size(); i++) {
                double angle1 = jointAngles1.get(i);
                double angle2 = jointAngles2.get(i);
                double angleChange = Math.abs(angle2 - angle1);
                if (angleChange > maxJointAngleChange) {
                    maxJointAngleChange = angleChange;
                }
            }
            // 计算夹持器长度的最大变化
            double gripperLengthChangeLimit = 0.001;
            double maxGripperLengthChange = 0;
            for (int i = 0; i < gripperLengths1.size(); i++) {
                double length1 = gripperLengths1.get(i);
                double length2 = gripperLengths2.get(i);
                double lengthChange = Math.abs(length2 - length1);
                if (lengthChange > maxGripperLengthChange) {
                    maxGripperLengthChange = lengthChange;
                }
            }
            // 根据四者中的最大变化计算插值步数
            int numSteps = (int) Math.ceil(Math.max(
                    Math.max(Math.abs(baseIncrementX / baseChangeLimit), Math.abs(baseIncrementY) / baseChangeLimit),
                    Math.max(maxJointAngleChange / JointAngleChangeLimit, maxGripperLengthChange/gripperLengthChangeLimit)
            ));
            return numSteps;
        }

        static class SearchNode {
            private SearchNode parent;
            private ArmConfig config;
            private double cost;
            private double heuristic;

            public SearchNode(ArmConfig config, double cost, SearchNode parent) {
                this.config = config;
                this.cost = parent.getCost() + heuristic(parent.config, config);
                this.parent = parent;
                this.heuristic = heuristic(this.config, goal);
            }
            public SearchNode(ArmConfig config, double cost) {
                this.config = config;
                this.cost = cost;
                this.parent = null;
            }
            public SearchNode getParent() {return parent;}
            public double getHeuristic() {return heuristic;}

            public ArmConfig getConfig() {
                return config;
            }
            public double getCost() {
                return cost;
            }

            public void updateCost(double newCost, double heuristic, SearchNode currentNode) {
            }
        }


    }

}
