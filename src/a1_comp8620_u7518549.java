import problem.ArmConfig;
import problem.ProblemSpec;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class a1_comp8620_u7518549 {

    public static void main(String[] args) throws IOException {

            if (args.length != 2) {
                System.out.println("Usage: java -jar a1_comp8620_u7518549.jar inputFileName outputFileName");
                return;
            }

            String inputFilename = args[0];
            String outputFilename = args[1];

            ProblemSpec ps = new ProblemSpec();

            try {
                ps.loadProblem(inputFilename);
            } catch (IOException e) {
                System.out.println("Error loading problem: " + e.getMessage());
                return;
            }
//         读取本地文件测试用代码，导出为jar时注释掉
//        ProblemSpec ps = new ProblemSpec();
//        String inputFilename = "testcases/gripper_4_joints.txt";
//        try {
//            ps.loadProblem(inputFilename);
//        } catch (IOException e) {
//            System.out.println("Error loading problem: " + e.getMessage());
//            return;
//        }
//        String outputFilename = "testcases/test_sol.txt";

        System.out.println("read problem:" + inputFilename);
        System.out.println("Robot Configuration:");
        System.out.println("Initial State: " + ps.getInitialState());
//        System.out.println(ps.getInitialState().getClass());
        System.out.println("Goal State: " + ps.getGoalState());
        System.out.println("Number of Joints: " + ps.getJointCount());
        System.out.println("Obstacles: " + ps.getObstacles());
        String hasGripper = ps.getInitialState().hasGripper()?"The arm has gripper" : "The arm does not have gripper";
        System.out.println(hasGripper);

        // 创建 PRM 对象
        PRM prm = new PRM(ps);
        // 生成路径规划结果，传递障碍物信息、起始点和目标点
        prm.generateRoadmap();

        // 获取生成的道路图
        List<ArmConfig> samples = prm.getSamples();
        PRM.PRMGraph prmGraph = prm.getRoadmap();
        List<ArmConfig> path = PRM.PRMSearch.search(prmGraph);

        System.out.println("The path the algorithm generates is:");
        if (path != null) {
            for (ArmConfig armConfig: path){
                System.out.println(armConfig.toString());
            }
        }
        // 对获得的path进行插值
        List<ArmConfig> interpolatedPath = new ArrayList<>();

    // 遍历现有的路径path中的相邻配置
        if (path != null) {
            for (int i = 0; i < path.size() - 1; i++) {
                ArmConfig config1 = path.get(i);
                ArmConfig config2 = path.get(i + 1);

                // 使用generatePath方法对config1和config2之间进行插值，并将插值后的路径添加到interpolatedPath中
                List<ArmConfig> interpolatedSegment = PRM.generatePath(config1, config2);
                interpolatedPath.addAll(interpolatedSegment);
            }
        }

        ps.setPath(interpolatedPath);
        ps.saveSolution(outputFilename);
    }
}
