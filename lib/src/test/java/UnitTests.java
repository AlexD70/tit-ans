import org.json.JSONWriter;
import org.junit.jupiter.api.Test;
import org.junit.platform.commons.util.ModuleUtils;
import titans.geometry.Point2d;
import titans.roads.Spline;

import java.io.File;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.Arrays;
import java.util.Optional;
import java.util.Scanner;
import java.util.Timer;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class UnitTests {
    // points used for testing
    public Point2d p1 = new Point2d(0.732, 2.901),
            p2 = new Point2d(-5.17, 3.3),
            p3 = new Point2d(4.87, -5),
            p4 = new Point2d(8.3, 18.33),
            p5 = new Point2d(-0.233, 1.5),
            p6 = new Point2d(-21, 3);

    // this test checks if the python output and the java output
    // are within an error margin of 0.00001 (set in the py source)
    @Test
    void testSplineBuild() throws IOException, InterruptedException {
        // get java output
        Instant start = Instant.now();
        Spline s = Spline.buildSpline6(p1, p4, p2, p5, p3, p6);
        Instant end = Instant.now();

        long elapsed = Duration.between(start, end).toNanos();

        // encode data to json
        StringBuffer jsonData = new StringBuffer();
        JSONWriter jsonifier = new JSONWriter(jsonData)
                .object()
                .key("points")
                .array();

        for(Point2d p : Arrays.asList(p1, p2, p3, p4, p5, p6)){
            jsonifier.array().value(p.getX()).value(p.getY()).endArray();
        }

        Double[] xcoeffs = s.getXCoeffs(), ycoeffs = s.getYCoeffs();
        jsonifier.endArray()
                .key("xcoef")
                .value(xcoeffs)
                .key("ycoef")
                .value(ycoeffs)
                .key("time")
                .value(elapsed)
                .endObject();

        System.out.print(jsonData);

        // call python and pass json data as sys.argv[1]
        ProcessBuilder pythonProc = new ProcessBuilder(
                "python", new File("src/test/python/path_solve_test.py").getAbsolutePath(), jsonData.toString()
        );
        Process proc = pythonProc.start();
        Scanner reader = new Scanner(proc.getInputStream());

        // print python output
        System.out.print("\n");
        while(reader.hasNextLine()) {
            System.out.println(reader.nextLine());
        }

        // wait for exit code and assert its value is 0
        proc.waitFor();
        assertEquals(0, proc.exitValue());
    }
}
