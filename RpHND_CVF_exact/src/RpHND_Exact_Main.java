import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.PriorityQueue;

import org.apache.commons.math3.util.Combinations;

import gurobi.GRB;
import gurobi.GRBException;
import gurobi.GRBModel;
import gurobi.GRBVar;
import model.KMedoids;
import model.Network;
import model.Node;
import model.Plot;
import model.Route;
import model.RoutingTree;
import model.Txt2Array;

public class RpHND_Exact_Main {
	public static double[][] failures, distances, fixedCosts, flows;
	public static int nVar, nConst, P, L, M;
	public static double alpha;
	public static Node[] nodes;
	public static Route[][][][] routes;
	public static Combinations hubCombs;
	public static List<Integer> potentialHubs;
	public static long timeLimit;
	public static boolean isExact;

	public static void main(String[] args) throws IOException,
			InterruptedException, GRBException {
		timeLimit = 5*1000;
		String timestamp = new SimpleDateFormat("yyyyMMdd-HH.mm.ss").format(new Date());
		PrintWriter out = new PrintWriter(new File("results_" + timestamp + ".txt"));
		
		int[] SIZE = {10,15,20,25};
		int[] P = { 3,5,7 };
		int[] L = { 0,1,2,3};
		double[] DISCOUNT = { 0.2,0.4,0.6 };
		double[] FAILURE = { 0.05, 0.1, 0.15 , 0.20 };

		// printing headers
		String header = "N,p,alpha,q,l,counter,hubs,obj,time,exact,time stamp,flows_bad,flows_indifferent,flows_good,nonhub_links,interhub_links,total_inter_hub_distance";
		System.out.println(header);
		out.append(header + "\r\n");
//		Plot myplot = new Plot("Test_Plot", Txt2Array.read("hubs.txt"," "), Txt2Array.read("spokes.txt"," "));
//		myplot.draw();
		for (int l : L)
			for (int n : SIZE)
				for (int p : P)
					for (double q : FAILURE)
						for (double d : DISCOUNT)	
							if (l < p){
								isExact = true;
								Network net = run(n, p, q, d, l);
								double temp1 = getHubsDispersion(net);
								double temp2 = getFlowsProportions2(net);
								String consoleOutput = "," + String.format("%.4f", temp1) + "," + String.format("%.4f", temp2);
								System.out.println(net.results + consoleOutput);
								out.append(net.results + consoleOutput + "\r\n");
							}
		
		
		/*String header = "N,p,alpha,q,l,instance,counter,hubs,obj,time,exact,time stamp,flows_bad,flows_indifferent,flows_good,nonhub_links,interhub_links,total_inter_hub_distance";
		int[] SIZE = { 20};
		int[] P = { 7 };
		int[] L = { 2 };
		double[] DISCOUNT = { 0.2 };
		double[] FAILURE = { 0.02, 0.04, 0.06 , 0.08, 0.1};
		
		for (int l : L)
			for (int n : SIZE)
				for (int p : P)
					for (double q : FAILURE)
						for (double d : DISCOUNT)	
							for (int i = 0 ; i < 10; i++){
								if (l < p){
									isExact = true;
									Network net = run(n, p, q, d, l, i);
									double[] temp1 = getFlowsProportions(net);
									double[] temp2 = getLinksProportions(net);
									double temp3 = getHubsDispersion(net);
									String consoleOutput = "," + String.format("%.4f", temp1[0]) + "," + String.format("%.4f", temp1[1]) + "," + String.format("%.4f", temp1[2]) + "," + String.format("%.4f", temp2[0]) + "," + String.format("%.4f", temp2[1]) + "," + String.format("%.4f", temp3);
									System.out.println(net.results + consoleOutput);
									out.append(net.results + consoleOutput + "\r\n");
								}
							}*/
							
		out.close();
	};

	public static Network run(int N, int p, double q, double d, int l)
			throws IOException, InterruptedException, GRBException {
		String result = "";
		failures = MyArray.read("Datasets/An_Failures/failures.txt");
		distances = MyArray.read("Datasets/CAB/CAB" + N + "/Distances.txt");
		nVar = distances.length;
		flows = MyArray.read("Datasets/CAB/CAB" + N + "/Flows.txt");
		fixedCosts = MyArray.read("Datasets/CAB/CAB" + N + "/fixedcharge.txt");
		P = p;
		L = l; // maximum number of failures
		alpha = d;
		nodes = new Node[nVar];
		routes = new Route[nVar][nVar][nVar][nVar];
		double startTime = System.currentTimeMillis();

		// --------------- setting node failures ------------
		/*for (int i = 0; i < nVar; i++)
			failures[i][0] = q;*/

		// --------------- initializing node objects ------------
		for (int i = 0; i < nVar; i++) {
			nodes[i] = new Node(i, failures[i][0]);
		}

		// ---------- initializing hub combinations -------------
		Combinations hubCombs = new Combinations(nVar, P);

		// ---------- run k-medoids algorithm to obtain a seed for hub
		// combinations and get an upper bound ------------
		int[] initialHubs = new int[P];
		List<Integer> intialHubsTemp = KMedoids.run("Datasets/CAB/CAB" + N
				+ "/Distances.txt", P);
		for (int i = 0; i < P; i++)
			initialHubs[i] = intialHubsTemp.get(i);

		/*
		 * HND traditionalHLP = new HND(distances, flows, p, alpha); int[]
		 * initialHubs = traditionalHLP.hubs;
		 */

		// ------------ Create a network based on the seed ---------------
		Network initialNetwork = getNetwork(initialHubs, L);

		double upperBound = initialNetwork.cost;
		Network bestNetwork = initialNetwork;
		int counter = 0;
		for (int[] hubComb : hubCombs) {
			double networkLowerBound = getNetworkUpperBound(hubComb, L, upperBound);
			if (networkLowerBound < upperBound) {
				Network network = getNetwork(hubComb, L);
				if (network.cost < upperBound) {
					upperBound = network.cost;
					bestNetwork = network;
				}
				counter++;
			}
		}
		result = N + "," + p + "," + d + "," + q + "," + l + "," 
				+ counter + ",";
		for (int i : bestNetwork.hubs)
			result += i + " ";
		result += "," + bestNetwork.cost;
		double finishTime = System.currentTimeMillis() - startTime;
		result += "," + finishTime + "," + isExact + "," + new SimpleDateFormat("yyyy/MM/dd-HH:mm:ss").format(new Date());
		bestNetwork.results  = result;
		return bestNetwork;
	}

	private static double getNetworkUpperBound(int[] hubComb, int l, double bestUB) {
		double upperBound = 0;
		for (int i = 0; i < nVar; i++)
			for (int j = i + 1; j < nVar; j++){
				upperBound += getLowerbound(i, j, hubComb, l) * flows[i][j];
				if (upperBound > bestUB)
					return upperBound;
			}				
		return upperBound;
	}

	private static Network getNetwork(int[] hubs, int l) {
		RoutingTree[][] routingTrees = new RoutingTree[nVar][nVar];
		for (int i = 0; i < nVar; i++) {
			for (int j = i + 1; j < nVar; j++) {
				double RTUB = getUpperbound(i, j, hubs, l); // the upper bound
															// obtained by the
															// greedy algorithm
				routingTrees[i][j] = getRoutingTree(i, j, hubs, l, RTUB);
			}
		}
		return new Network(hubs, routingTrees, flows);
	}

	/**
	 * Prints the basic variables.
	 * 
	 * @param model
	 * @throws GRBException
	 */
	public static void printSol(GRBModel model, String flag)
			throws GRBException {
		if (flag.equals("nonzeros")) {
			for (GRBVar var : model.getVars()) {
				double value = var.get(GRB.DoubleAttr.X);
				if (value != 0)
					System.out.println(var.get(GRB.StringAttr.VarName) + ": "
							+ value + ": " + var.get(GRB.DoubleAttr.Obj));
			}
		} else if (flag.equals("all")) {
			for (GRBVar var : model.getVars())
				System.out.println(var.get(GRB.StringAttr.VarName) + ": "
						+ var.get(GRB.DoubleAttr.X) + ": "
						+ var.get(GRB.DoubleAttr.Obj));
		}
	}

	/**
	 * returns an upper bound on the best routing tree of depth l from i to j
	 * using hub combination hList
	 * 
	 * @param i
	 * @param j
	 * @param hList
	 * @param l
	 * @return double
	 */
	public static double getUpperbound(int i, int j, int[] hList, int l) {
		// Update the nodes in the hubsList by setting the isHub to true
		for (int h : hList)
			nodes[h].isHub = true;

		// instantiating the output
		RoutingTree output = new RoutingTree((int) Math.pow(2, l + 1) - 1);

		// generating list of feasible routes between the origin and the
		// destination.
		PriorityQueue<Route> feasibleRoutes = RoutingTree.getFeasibleRoutes2(i,
				j, hList, nodes, routes, distances, alpha);

		int cntr = 0; // counter
		int lastIndex = (int) (Math.pow(2, l) - 2); // the last index of the
													// second last level of the
													// tree

		Route selectedRoute = feasibleRoutes.poll();
		output.routes[0] = selectedRoute;
		output.usedHubs.put(0, new ArrayList<Node>());

		while (!feasibleRoutes.isEmpty() && cntr <= lastIndex) {
			// List of feasible routes to select left child node from.
			PriorityQueue<Route> feasibleRoutes1 = new PriorityQueue<Route>(
					feasibleRoutes);
			PriorityQueue<Route> feasibleRoutes2 = new PriorityQueue<Route>(
					feasibleRoutes);

			if (output.routes[cntr] != null
					&& !output.routes[cntr].i.equals(output.routes[cntr].k)
					&& !isFinal(output.routes[cntr])) {
				// List of feasible routes to select right child node from.
				ArrayList<Node> usedHubs1 = new ArrayList<Node>(
						output.usedHubs.get(cntr));

				usedHubs1.add(output.routes[cntr].k); // adding parent node's
														// first hub to the list
				int leftNodeIndex = 2 * cntr + 1;
				while (output.routes[leftNodeIndex] == null) {
					selectedRoute = feasibleRoutes1.poll();

					if (!usedHubs1.contains(selectedRoute.k)
							&& !usedHubs1.contains(selectedRoute.m)) {
						output.routes[2 * cntr + 1] = selectedRoute;
						output.usedHubs.put(2 * cntr + 1, usedHubs1);
					}
				}
			}

			if (output.routes[cntr] != null
					&& !output.routes[cntr].j.equals(output.routes[cntr].m)
					&& !isFinal(output.routes[cntr])
					&& !output.routes[cntr].k.equals(output.routes[cntr].m)) {
				ArrayList<Node> usedHubs2 = new ArrayList<Node>(
						output.usedHubs.get(cntr)); // make
													// a
													// list
													// of
													// parent
													// node's
													// used
													// hubs.
				usedHubs2.add(output.routes[cntr].m); // adding parent node's
														// first hub to the list
				int rightNodeIndex = 2 * cntr + 2;
				while (output.routes[rightNodeIndex] == null) {
					selectedRoute = feasibleRoutes2.poll();
					if (!usedHubs2.contains(selectedRoute.k)
							&& !usedHubs2.contains(selectedRoute.m)) {
						output.routes[2 * cntr + 2] = selectedRoute;
						output.usedHubs.put(2 * cntr + 2, usedHubs2);
					}
				}
			}
			cntr++;
		}

		// switching node is hubList back to spokes
		for (Integer n : hList)
			nodes[n].isHub = false;

		// updating the value of the tree
		output.updateValue();

		return output.value;
	}

	/**
	 * computes a lower bound on the expected cost of the routing tree going
	 * from i to j give hub combination hList with up to l simultaneous
	 * failures.
	 * 
	 * @param i
	 * @param j
	 * @param hList
	 * @param l
	 * @return double - lower bound
	 */
	public static double getLowerbound(int i, int j, int[] hList, int l) {
		// Update the nodes in the hubsList by setting the isHub to true
		for (int h : hList)
			nodes[h].isHub = true;

		// instantiating the output
		RoutingTree output = new RoutingTree((int) Math.pow(2, l + 1) - 1);

		// generating list of feasible routes between the origin and the
		// destination.
		PriorityQueue<Route> feasibleRoutes = RoutingTree.getFeasibleRoutes2(i,
				j, hList, nodes, routes, distances, alpha);

		int cntr = 0; // counter
		int lastIndex = (int) (Math.pow(2, l) - 2); // the last index of the
													// second last level of the
													// tree

		Route bestRoute = feasibleRoutes.poll();
		output.routes[0] = bestRoute;
		output.usedHubs.put(0, new ArrayList<Node>());

		while (cntr <= lastIndex) {
			if (output.routes[cntr] != null
					&& !output.routes[cntr].i.equals(output.routes[cntr].k)
					&& !isFinal(output.routes[cntr])) {
				// List of feasible routes to select right child node from.
				ArrayList<Node> usedHubs1 = new ArrayList<Node>(
						output.usedHubs.get(cntr));
				usedHubs1.add(output.routes[cntr].k); // adding parent node's
														// first hub to the list
				output.routes[2 * cntr + 1] = bestRoute;
				output.usedHubs.put(2 * cntr + 1, usedHubs1);
			}

			if (output.routes[cntr] != null
					&& !output.routes[cntr].j.equals(output.routes[cntr].m)
					&& !isFinal(output.routes[cntr])
					&& !output.routes[cntr].k.equals(output.routes[cntr].m)) {
				// make a list of parent node's used hubs.
				ArrayList<Node> usedHubs2 = new ArrayList<Node>(
						output.usedHubs.get(cntr));
				usedHubs2.add(output.routes[cntr].m); // adding parent node's
														// first hub to the list
				output.routes[2 * cntr + 2] = bestRoute;
				output.usedHubs.put(2 * cntr + 2, usedHubs2);

			}
			cntr++;
		}

		// switching node is hubList back to spokes
		for (Integer n : hList)
			nodes[n].isHub = false;

		// updating the value of the tree
		output.updateValue();
		return output.value;
	}

	/**
	 * returns true if a route requires a back up.
	 * 
	 * @param r
	 * @return boolean
	 */
	private static boolean isFinal(Route r) {
		boolean output = false;
		if (r.k.equals(r.m)) {
			if (r.k.equals(r.i) || r.m.equals(r.j))
				output = true;
		} else if (r.i.equals(r.k) && r.j.equals(r.m)) {
			output = true;
		}
		return output;
	}

	/**
	 * Returns the optimal routing tree of depth l from i to j using hub
	 * combination hList
	 * 
	 * @param i
	 * @param j
	 * @param hList
	 * @param l
	 * @param UB
	 * @return RoutingTree
	 */
	public static RoutingTree getRoutingTree(int i, int j, int[] hList, int l,
			double UB) {
		double bestRTValue = GRB.INFINITY;
		RoutingTree bestRT = new RoutingTree((int) Math.pow(2, l + 1) - 1);

		// Update the nodes in the hubsList by setting the isHub to true
		for (Integer n : hList)
			nodes[n].isHub = true;

		// generating list of feasible routes between the origin and the
		// destination.
		ArrayList<Route> feasibleRoutes = RoutingTree.getFeasibleRoutes(i, j,
				hList, nodes, routes, distances, alpha);

		// instantiating unexplored RoutingTreeNodes
		ArrayList<RoutingTree> unexploredNodes = new ArrayList<RoutingTree>();

		// initializing a RoutingTreeNode for each feasible routes.
		for (Route r : feasibleRoutes) {
			RoutingTree rt = new RoutingTree(
					new Route[(int) Math.pow(2, l + 1) - 1], r, feasibleRoutes,
					L, UB);
			if (rt.isComplete()) {
				if (rt.value < bestRTValue) {
					bestRT = rt;
					bestRTValue = bestRT.value;
				}
			} else if (!rt.pruned)
				unexploredNodes.add(rt);
		}

		double startTime = System.currentTimeMillis();
		while (!unexploredNodes.isEmpty()) {
			if (System.currentTimeMillis() - startTime > timeLimit  && bestRTValue != GRB.INFINITY){
				isExact = false;
				break;
			}
			RoutingTree currentNode = unexploredNodes.get(unexploredNodes
					.size() - 1);
			unexploredNodes.remove(unexploredNodes.size() - 1);

			if (!currentNode.isComplete()) {
				int childNodeInd = currentNode.unexploredNodes.get(0);
				int parentNodeInd = (int) Math.floor((childNodeInd - 1) / 2);

				boolean flag = true; // left node
				if (childNodeInd % 2 == 0)
					flag = false; // right node

				ArrayList<Route> avlRoutes = currentNode.availableRoutes
						.get(parentNodeInd);
				if ( flag ) {
					avlRoutes = RoutingTree.getAvlRoutes(avlRoutes,
							currentNode.routes[parentNodeInd].k);
				} else {
					avlRoutes = RoutingTree.getAvlRoutes(avlRoutes,
							currentNode.routes[parentNodeInd].m);
				}
				for (Route r : avlRoutes) {
					RoutingTree rt = new RoutingTree(currentNode, childNodeInd,
							r, avlRoutes, L, UB);
					// Check for the best value and the best routing-tree
					if (rt.isComplete()) {
						if (rt.value < bestRTValue) {
							bestRT = rt;
							bestRTValue = bestRT.value;
							if (bestRTValue < UB)
								UB = bestRTValue;
						}
					} else if (!rt.pruned)
						unexploredNodes.add(rt);
				}
			}
		}
		// switching node is hubList back to spokes
		for (Integer n : hList)
			nodes[n].isHub = false;

		return bestRT;
	}
	
	private static double[] getFlowsProportions(Network net){
		double[] output = {0,0,0};
		double totalFlow = 0;
		int index;		
		
		for (int i = 0 ; i < nVar ; i++){
			for (int j = i+1 ; j < nVar ; j++){
				totalFlow += flows[i][j];
				Route r = net.routingTrees[i][j].routes[0];
				if ( !r.k.equals(r.m) ) //iijj or ikmj or iimj or ikjj
					index = 2;
				else {
					if ( !r.i.equals(r.k) && !r.j.equals(r.m)) //ikkj
						index = 0;
					else	// iiij or ijjj
						index = 1;
				}									
				output[index] += flows[r.i.ID][r.j.ID];
			}
		}
		
		for (int i = 0 ; i < output.length ; i++ )
			output[i] = output[i]/totalFlow;
		return output;
	}

	private static double[] getLinksProportions(Network net){
		double[] output = {0,0};
		double totalDistance = 0;
		
		for (int i = 0 ; i < nVar ; i++){
			for (int j = i+1 ; j < nVar ; j++){
				Route r = net.routingTrees[i][j].routes[0];
				output[0] += distances[r.i.ID][r.k.ID] + distances[r.m.ID][r.j.ID];
				output[1] += distances[r.k.ID][r.m.ID];
			}
		}
		
		for (int i = 0 ; i < output.length ; i++ )
			totalDistance += output[i];
		
		for (int i = 0 ; i < output.length ; i++ )
			output[i] /= totalDistance;
		
		return output;
	}
	
	/**
	 *  returns percentage of flows that go through inter-hub links.
	 * @param Network
	 * @return double
	 */
	public static double getFlowsProportions2(Network net){
		double numinator = 0;
		double denuminator = 0;
		for ( int i = 0 ; i < nVar ; i++ )
			for (int j = i+ 1 ; j < nVar ; j++ ){
				Route r = net.routingTrees[i][j].routes[0];
				numinator += flows[r.i.ID][r.j.ID] * distances[r.k.ID][r.m.ID];
				denuminator += flows[r.i.ID][r.j.ID] * ( distances[r.i.ID][r.k.ID] + distances[r.k.ID][r.m.ID] + distances[r.m.ID][r.j.ID] );
			}
		return numinator/denuminator;
	}
	
	/**
	 * return total distances of pair-wise hub distances in a network
	 *
	 * @param Network
	 * @return double
	 */
	private static double getHubsDispersion(Network net){
		double output = 0;
		for (int i : net.hubs)
			for (int j : net.hubs)
				output += distances[i][j];
		return output;
	}
}
