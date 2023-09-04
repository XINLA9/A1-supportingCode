import problem.ArmConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.util.*;

import static java.lang.Math.max;

public class PRM {

    private static final int NUM_SAMPLES = 10000; // sample number
    private static final double NEIGHBOR_RADIUS = 0.1; // Neighborhood radius
    private static final int NUM_NEIGHBORS = 10; // Neighborhood points

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
        generateSamples();
        // 2. Build the roadmap
        buildRoadmap();
    }

    /**
     * Generates random samples and adds them to the sample list.
     *
     */
    private void generateSamples() {
        samples.add(init);
        samples.add(goal);
        Random rand = new Random();
        int iteration = 0;
        while (iteration < NUM_SAMPLES) {
            ArmConfig randomConfig = getRandomSampleConfiguration();
            // Check sampling points for collisions or out-of-bounds
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
     *
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
         *
         * @param withGripper True if the configuration should include gripper information, false otherwise.
         * @param jointCount The number of joint angles to generate.
         * @return A formatted string representing the random configuration.
         */
        public static String gerRandomString(boolean withGripper, int jointCount) {
            Random rand = new Random();
            // Generate base coordinates
            double baseX = MIN_X + rand.nextDouble() * (MAX_X - MIN_X);
            double baseY = MIN_Y + rand.nextDouble() * (MAX_Y - MIN_Y);
            StringBuilder configStr = new StringBuilder();
            // Coordinates of the center of the base
            configStr.append(String.format("%.2f %.2f ", baseX, baseY));
            // Generate joint angles
            for (int i = 0; i < jointCount; i++) {
                double jointAngle = MIN_JOINT_ANGLE + rand.nextDouble() * (MAX_JOINT_ANGLE - MIN_JOINT_ANGLE);
                configStr.append(String.format("%.2f ", jointAngle));
            }
            // If you need to generate a configuration with grippers, generate the gripper mid-section lengths
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
     * Finds the closest neighbor to the given configuration and adds up to the specified number of neighbors。
     *
     * @param config To find a neighbor's configuration
     * @param maxNeighbors Maximum number of neighborss
     * @return A list of the most recent neighbors, containing at most maxNeighbors.
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
        // Sort neighbors by proximity to distance
        neighbors.sort((c1, c2) -> Double.compare(calculateConfigDistance(config, c1), calculateConfigDistance(config, c2)));
        return neighbors;
    }
    /**
     * Check that it is possible to connect two ArmConfig
     * to ensure that the path does not turn into an obstacle or fail to materialize.
     *
     * @param config1 first config to connect
     * @param config2 second config to connect
     * @return Is it possible to connect two ArmConfigs
     */
    private boolean isValidConnection(ArmConfig config1, ArmConfig config2) {
        // Get the path between ArmConfig
        List<ArmConfig> path = generatePath(config1, config2);
        // Check that each configuration on the path intersects with an obstacle
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
     * Calculates the distance between two configurations. The distance consists of the maximum change
     * time between the joint angle, the length of the gripper mid-section and the base coordinates.
     *
     * @param config1 First configuration
     * @param config2 Second configuration
     * @return Distance between two configurations, measured in terms of maximum change time
     */
    private double calculateConfigDistance(ArmConfig config1, ArmConfig config2) {
        Point2D base1 = config1.getBaseCenter();
        Point2D base2 = config2.getBaseCenter();

        return base1.distance(base2);
    }


    /**
     * Generates an interpolated path between two ArmConfig.
     *
     * @param config1 Starting ArmConfig
     * @param config2 Target ArmConfig
     * @return Generated paths, including start and destination configurations
     */
    public static List<ArmConfig> generatePath(ArmConfig config1, ArmConfig config2) {
        List<ArmConfig> path = new ArrayList<>();
        if (config1.getJointAngles().size() != config2.getJointAngles().size()) {
            throw new IllegalArgumentException("Configurations have different joint counts.");
        }
        // Get information about the starting and target configurations
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

        // Base translation limitation
        double baseChangeLimit = 0.001 / 2;
        // 计算关节角度变化限制
        double JointAngleChangeLimit = 0.10 / 2;
        double maxJointChange = Collections.max(jointAngleDifferences);

        // Calculating the Limit of Change in Gripper Length
        double gripperLengthChangeLimit = 0.001 / 2;
        double maxGripperChange = 0;
        if (hasGripper){
            maxGripperChange = Collections.max(gripperLengthDifference);
        }


        // Calculate the number of interpolation steps based on the maximum change among the four
        int numSteps = (int) Math.ceil(
                Math.max(Math.abs(baseDifference / baseChangeLimit),
                        Math.max(maxJointChange / JointAngleChangeLimit, maxGripperChange/gripperLengthChangeLimit)
        ));

        if (numSteps < 2) {
            numSteps = 2; // At least two steps are required
        }

        // Calculate the increment for each basic step
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

            // Calculate interpolation of joint angles
            List<Double> interpolatedJointAngles = new ArrayList<>();
            for (int i = 0; i < jointAngles1.size(); i++) {
                double angle = jointAngles1.get(i);
                double interpolatedAngle = angle + jointIncrement.get(i) * step;
                interpolatedJointAngles.add(interpolatedAngle);
            }

            // Calculating the interpolated value of the gripper length
            List<Double> interpolatedGripperLengths = new ArrayList<>();
            for (int i = 0; i < gripperLengths1.size(); i++) {
                double length1 = gripperLengths1.get(i);
                double length2 = gripperLengths2.get(i);
                double interpolatedLength = length1 + gripperIncrement.get(i) * step;
                interpolatedGripperLengths.add(interpolatedLength);
            }

            // Creating Interpolation Configurations
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
            // Use the priority queue to sort by evaluation function
            PriorityQueue<SearchNode> openQueue = new PriorityQueue<>(new Comparator<SearchNode>() {
                @Override
                public int compare(SearchNode node1, SearchNode node2) {
                    // Comparison based on cost + heuristic
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

            // Initialize the start node
            SearchNode startNode = new SearchNode(start, 0);
            openQueue.offer(startNode);
            nodeMap.put(start, startNode);
            SearchNode currentNode = null;

            while (!openQueue.isEmpty()) {
                currentNode = openQueue.poll();

                ArmConfig currentConfig = currentNode.getConfig();

                if (currentConfig.equals(goal)) {
                    // Find the target configuration and backtrack the path
                    System.out.println("Find a path to goal!");
                    return reconstructPath(currentNode);
                }

                closedList.add(currentConfig);

                for (ArmConfig neighbor : neighbors.get(currentConfig)) {
                    if (closedList.contains(neighbor)) {
                        continue; // Nodes that have already been explored, skip
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
            System.out.println("Can't not find a path to goal!");
            return null;
        }


        /**
         * Reverse reconstruction of the path from the current node.
         *
         * @param node final node
         * @return Path from the start node to the final node
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
         * Calculates the distance between two ArmConfig objects.
         *
         * @param armConfig1 The first ArmConfig object
         * @param armConfig2 The second ArmConfig object
         * @return Distance between two ArmConfig objects
         */
        private static double cost(ArmConfig armConfig1, ArmConfig armConfig2){
            // Get the base position for both configurations
            Point2D base1 = armConfig1.getBaseCenter();
            Point2D base2 = armConfig2.getBaseCenter();
            // Get the joint angles for both configurations
            List<Double> jointAngles1 = armConfig1.getJointAngles();
            List<Double> jointAngles2 = armConfig2.getJointAngles();
            // Calculating Euclidean distances for base locations
            double baseDistance = base1.distance(base2);
            // Calculate the difference between joint angles
            double jointAngleDifference = 0.0;
            for (int i = 0; i < jointAngles1.size(); i++) {
                double angle1 = jointAngles1.get(i);
                double angle2 = jointAngles2.get(i);
                double angleDiff = Math.abs(angle2 - angle1);
                jointAngleDifference += angleDiff;
            }
            // Initializing Gripper Length Differences
            double gripperLengthDifference = 0.0;
            if (armConfig1.hasGripper()) {
                // If there are grippers, calculate the difference between the gripper lengths
                List<Double> gripperLengths1 = armConfig1.getGripperLengths();
                List<Double> gripperLengths2 = armConfig2.getGripperLengths();

                for (int i = 0; i < gripperLengths1.size(); i++) {
                    double length1 = gripperLengths1.get(i);
                    double length2 = gripperLengths2.get(i);
                    double lengthDiff = Math.abs(length2 - length1);
                    gripperLengthDifference += lengthDiff;
                }
            }
            // Combining base distance, joint angle difference and gripper length difference into total distance

            return 0.0;
        }
        private static double heuristic(ArmConfig config1, ArmConfig config2){
            Point2D base1 = config1.getBaseCenter();
            Point2D base2 = config2.getBaseCenter();
            List<Double> jointAngles1 = config1.getJointAngles();
            List<Double> jointAngles2 = config2.getJointAngles();
            List<Double> gripperLengths1 = config1.getGripperLengths();
            List<Double> gripperLengths2 = config2.getGripperLengths();
            // Calculate the increment of the base translation
            double baseChangeLimit = 0.001;
            double baseIncrementX = (base2.getX() - base1.getX());
            double baseIncrementY = (base2.getY() - base1.getY());
            // Calculate the maximum change in joint angle
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
            // Calculate the maximum change in gripper length
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
            // Calculate the number of interpolation steps based on the maximum change among the four
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
