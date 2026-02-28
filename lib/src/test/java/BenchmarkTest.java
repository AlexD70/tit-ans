import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import titans.roads.ConstraintSetXY;

import java.time.Duration;
import java.time.Instant;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class BenchmarkTest {
    @Test @Tag("benchmark")
    void benchmarkProfileGeneration(){
        MotionProfileTests profileTests = new MotionProfileTests();
        ConstraintSetXY constr = profileTests.constr;

        System.out.println("nr, ms");
        for(int i = 0; i < 100; i ++) {
            Instant start = Instant.now();
            profileTests.buildTestProfile(constr, 0, 0, 0, 2000);
            Instant end = Instant.now();

            long millisElapsed = Duration.between(start, end).toMillis();
            System.out.printf("%d, %d%n", i, millisElapsed);
        }

        assertTrue(true);
    }
}
