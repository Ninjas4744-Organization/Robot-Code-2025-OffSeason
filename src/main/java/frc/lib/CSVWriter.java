package frc.lib;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class CSVWriter {
    public static void writeCsv(String xName, String yName, List<Double> xValues, List<Double> yValues, String filename) {
        if (xValues.size() != yValues.size()) {
            System.err.println("X and Y lists must be the same size!");
            return;
        }

        String path = "/media/sda1/CSV/";

        try (FileWriter writer = new FileWriter(path + filename)) {
            writer.write(xName + "," + yName + "\n"); // CSV header
            for (int i = 0; i < xValues.size(); i++) {
                writer.write(xValues.get(i) + "," + yValues.get(i) + "\n");
            }
            System.out.println("CSV written to " + path + filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
