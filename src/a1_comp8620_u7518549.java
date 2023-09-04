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

        System.out.println("\nRead problem file: "+inputFilename);
        System.out.println("Arm Configuration:");
        System.out.println("Initial State: " + ps.getInitialState());
        System.out.println("Goal State: " + ps.getGoalState());
        System.out.println("Number of Joints: " + ps.getJointCount());
        System.out.println("Obstacles: " + ps.getObstacles());
        String hasGripper = ps.getInitialState().hasGripper()?"The arm has gripper" : "The arm does not have gripper";
        System.out.println(hasGripper);

        // Creating PRM Objects
        PRM prm = new PRM(ps);
        // Generate path planning results, communicating obstacle information, start and goal points
        prm.generateRoadmap();

        // Get the generated road map
        List<ArmConfig> samples = prm.getSamples();
        PRM.PRMGraph prmGraph = prm.getRoadmap();
        List<ArmConfig> path = PRM.PRMSearch.search(prmGraph);

        System.out.println("The path the algorithm generates is:");
        if (path != null) {
            for (ArmConfig armConfig: path){
                System.out.println(armConfig.toString());
            }
        }
        // Interpolate the obtained path
        List<ArmConfig> interpolatedPath = new ArrayList<>();

    // Iterate over neighboring configurations in an existing path
        if (path != null) {
            for (int i = 0; i < path.size() - 1; i++) {
                ArmConfig config1 = path.get(i);
                ArmConfig config2 = path.get(i + 1);

                // Use the generatePath method to interpolate between config1 and config2
                // and add the interpolated path to the interpolatedPath
                List<ArmConfig> interpolatedSegment = PRM.generatePath(config1, config2);
                interpolatedPath.addAll(interpolatedSegment);
            }
        }

        ps.setPath(interpolatedPath);
        ps.saveSolution(outputFilename);
    }
}
