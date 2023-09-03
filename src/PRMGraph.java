import problem.ArmConfig;

import java.util.*;

public class PRMGraph {
    private final Map<ArmConfig, List<ArmConfig>> graph;

    public PRMGraph() {
        graph = new HashMap<>();
    }

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

    // 添加边连接两个配置
    public void addEdge(ArmConfig config1, ArmConfig config2) {
        if (!hasEdge(config1, config2)) {
            List<ArmConfig> neighbors1 = getNeighbors(config1);
            neighbors1.add(config2);

            List<ArmConfig> neighbors2 = getNeighbors(config2);
            neighbors2.add(config1);
        }
    }
}