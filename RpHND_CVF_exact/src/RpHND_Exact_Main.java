import gurobi.GRB;
import gurobi.GRBException;
import gurobi.GRBModel;
import gurobi.GRBVar;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

import model.HND;
import model.KMedoids;
import model.Network;
import model.Node;
import model.Route;
import model.RoutingTree;

import org.apache.commons.math3.util.Combinations;

public class RpHND_Exact_Main {
	public static double[][] failures, distances, fixedCosts, flows;
	public static int nVar, nConst, P, L, M;
	public static double alpha;
	public static Node[] nodes;
	public static Route[][][][] routes;
	public static Combinations hubCombs;
	public static List<Integer> potentialHubs;

	public static void main(String[] args) throws IOException,
			InterruptedException, GRBException {
		int[] SIZE = { 10, 15, 20, 25 };
		int[] P = { 3, 5, 7 };
		int[] L = { 0, 1, 2, 3 };
		double[] DISCOUNT = { 0.2, 0.4, 0.6 };
		double[] FAILURE = { 0, 0.05, 0.1, 0.15, 0.2 };

		for (int l : L)
			for (int n : SIZE)
				for (int p : P)
					for (double q : FAILURE)
						for (double d : DISCOUNT)							
							if (l < p)
								run(n, p, q, d, l);
	};

	public static void run(int N, int p, double q, double d, int l)
			throws IOException, InterruptedException, GRBException {
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
		for (int i = 0; i < nVar; i++)
			failures[i][0] = q;

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
			double networkLowerBound = getNetworkLowerBound(hubComb, L);
			if (networkLowerBound < upperBound) {
				Network network = getNetwork(hubComb, L);
				if (network.cost < upperBound) {
					upperBound = network.cost;
					bestNetwork = network;
				}
				counter++;
			}
		}
		System.out.print(N + "," + p + "," + d + "," + q + "," + l + ","
				+ counter + ",");
		for (int i : bestNetwork.hubs)
			System.out.print(i + " ");
		System.out.print("," + bestNetwork.cost);
		double finishTime = System.currentTimeMillis() - startTime;
		System.out.println(" , Time: ," + finishTime);
	}

	private static double getNetworkLowerBound(int[] hubComb, int l) {
		double lowerBound = 0;
		for (int i = 0; i < nVar; i++)
			for (int j = i + 1; j < nVar; j++)
				lowerBound += getLowerbound(i, j, hubComb, l) * flows[i][j];
		return lowerBound;
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

		// initializing a RoutingTreeNode for each feasible route.
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

		while (!unexploredNodes.isEmpty()) {
			RoutingTree currentNode = unexploredNodes.get(unexploredNodes
					.size() - 1);
			unexploredNodes.remove(unexploredNodes.size() - 1);

			if (!currentNode.isComplete()) {
				int childNodeInd = currentNode.unexploredNodes.get(0);
				int parentNodeInd = (int) Math.floor((childNodeInd - 1) / 2);

				String flag = "left";
				if (childNodeInd % 2 == 0)
					flag = "right";

				ArrayList<Route> avlRoutes = currentNode.availableRoutes
						.get(parentNodeInd);
				if (flag.equals("left")) {
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
}
