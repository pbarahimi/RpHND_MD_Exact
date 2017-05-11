package model;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class Txt2Array {	
	
  @SuppressWarnings("resource")
public static double[][] read(String fileName, String delimiter) throws IOException {
    
    Scanner scanner = new Scanner(new File(fileName)).useDelimiter("\r\n");
    int arrayLength = 0;
    List<double[]> tempList = new ArrayList<double[]>();
    while(scanner.hasNext()){
    	arrayLength++;
    	String[] line = scanner.next().split(delimiter);
    	double[] tempLine = new double[line.length];
    	for (int i = 0 ; i < line.length ; i++ ){
    		tempLine[i] = Double.parseDouble(line[i]);
    	}
    	tempList.add(tempLine);
    }
    scanner.close();
    double[][] output = new double[arrayLength][tempList.size()];
    for (int i = 0 ; i < arrayLength ; i ++)
    	output[i] = tempList.get(i);    
    
    return output;
  }
}