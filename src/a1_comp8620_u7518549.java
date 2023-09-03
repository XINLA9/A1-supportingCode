import problem.ArmConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;

public class a1_comp8620_u7518549 {

    public static void main(String[] args) throws IOException {

        ProblemSpec ps = new ProblemSpec();

        String inputFilename = "testcases/4_joints.txt";

        try {
            ps.loadProblem(inputFilename);
        } catch (IOException e) {
            System.out.println("Error loading problem: " + e.getMessage());
            return;
        }
        System.out.println("Robot Configuration:");
        System.out.println("Initial State: " + ps.getInitialState());
//        System.out.println(ps.getInitialState().getClass());
        System.out.println("Goal State: " + ps.getGoalState());
        System.out.println("Number of Joints: " + ps.getJointCount());
        System.out.println("Obstacles: " + ps.getObstacles());

        // 创建 PRM 对象
        PRM prm = new PRM(ps);
        // 生成路径规划结果，传递障碍物信息、起始点和目标点
        prm.generateRoadmap();

        // 获取生成的道路图
        List<ArmConfig> samples = prm.getSamples();
        PRMGraph prmGraph = prm.getRoadmap();

        // 打印生成的道路图
        System.out.println("Generated Roadmap:");
        for (ArmConfig config : samples) {
            System.out.println(config);
        }
        ps.setPath(samples);
        String outputFile = inputFilename + "_sol";
        ps.saveSolution(outputFile);
    }
}
