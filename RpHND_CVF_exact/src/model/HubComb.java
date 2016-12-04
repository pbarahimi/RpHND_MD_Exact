package model;


import java.util.ArrayList;
import java.util.List;

public class HubComb {
	public int ID;
	public final List<Node> hubs;
	public final List<Node> spokes;
	public final double fixedCharge;
	
	public HubComb(int ID, List<Node> hubs, List<Node> spokes, double[][] fixedCosts){
		this.ID = ID;
		this.hubs = new ArrayList<Node>(hubs);
		this.spokes = new ArrayList<Node>(spokes);
		this.fixedCharge = getFixedCharge(fixedCosts);
	}
	
	public HubComb(int ID, int[] hubs, List<Node> nodes, double[][] fixedCosts){
		this.ID = ID;
		this.hubs = new ArrayList<Node>();
		for (int i = 0 ; i < hubs.length ; i++){
			this.hubs.add(nodes.get(hubs[i]));					
		}
		
		this.spokes = new ArrayList<Node>(nodes);
		this.spokes.removeAll(this.hubs);
		
		this.fixedCharge = getFixedCharge(fixedCosts);
	}
	
	private double getFixedCharge(double[][] fixedCosts){
		double output = 0;
		for (Node n : this.hubs){
			output += fixedCosts[n.ID][0];
		}
		return output;
	}
	
	@Override	
	public String toString(){
		String output = ID + " - hubs: ";
		for (Node i : hubs)
			output = output.concat(i.ID+",");
		/*output = output.concat(" - spokes: ");
		for (Node i : spokes)
			output = output.concat(i.ID+",");*/
		output = output.substring(0, output.length()-1);
		
		return output;// + " - F=" + this.fixedCharge;
	}
}
