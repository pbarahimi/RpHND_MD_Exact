package model;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Shape;
import java.io.IOException;

import javax.swing.JPanel;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.XYDataItem;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;
import org.jfree.util.ShapeUtilities;

public class Plot extends ApplicationFrame {
	public final XYSeriesCollection dataSeries;
	
	public Plot(String chartName, double[][] hubs, double[][] spokes) throws IOException {
        super(chartName);
        
        this.dataSeries = new XYSeriesCollection();
        this.dataSeries.addSeries(getSeries("hubs.txt"," ","Hubs"));
        this.dataSeries.addSeries(getSeries("spokes.txt"," ","Spokes"));
        
        JPanel jpanel = createDemoPanel();
        jpanel.setPreferredSize(new Dimension(640, 480));
        add(jpanel);
        
        
    }

    private JPanel createDemoPanel() throws IOException {
        JFreeChart jfreechart = ChartFactory.createScatterPlot(
            "Network Configuration", "X", "Y", this.dataSeries,
            PlotOrientation.VERTICAL, true, true, false);
        XYPlot plot = (XYPlot) jfreechart.getPlot();
        XYItemRenderer renderer = plot.getRenderer();
        renderer.setSeriesPaint( 0 , Color.RED );
        renderer.setSeriesShape(0, ShapeUtilities.createDiamond(5));
        renderer.setSeriesPaint( 1 , Color.cyan );
        renderer.setSeriesShape(1, ShapeUtilities.createRegularCross(3, 3));
        renderer.setBaseItemLabelsVisible(true);
        plot.setRenderer( renderer );        
        return new ChartPanel(jfreechart);
    }

    private static XYSeries getSeries(String fileName, String delimiter, String seriesName) throws IOException{
    	XYSeries output = new XYSeries(seriesName);
    	double[][] data = Txt2Array.read(fileName,delimiter);
    	for (int i = 0 ; i < data.length ; i++ )
    		output.add(data[i][0], data[i][1]);
    	return output;
    }
    
    public void draw() throws IOException {
        this.pack();
        RefineryUtilities.centerFrameOnScreen(this);
        this.setVisible(true);
    }
}
