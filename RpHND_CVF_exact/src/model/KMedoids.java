package model;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class KMedoids { 	

	public static List<Integer> run ( String filePath, int k ) throws IOException, InterruptedException {		
		ProcessBuilder pb = new ProcessBuilder("cmd",
				"/c",
				"python",
				"lib/KM.py",
				filePath,
				k + "");
		pb.redirectErrorStream(true);
		Process pr = pb.start();
		BufferedReader reader = new BufferedReader(new InputStreamReader(
				pr.getInputStream()));
		String centers = reader.readLine();
		pr.waitFor(5, TimeUnit.MINUTES);
		centers = centers.substring(1, centers.length()-1);
		String[] outputString = centers.split(", ");
		ArrayList<Integer> output = new ArrayList<>();
		for (int i = 0 ; i < outputString.length ; i++)
			output.add( Integer.parseInt(outputString[i]) );
		return output;
	}
	
}
