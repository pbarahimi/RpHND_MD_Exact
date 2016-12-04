package model;

import java.util.ArrayList;
import java.util.List;

public class NodeList {
	public List<Node> list = new ArrayList<Node>();
	
	public NodeList(){		
	};
	
	public NodeList(ArrayList<Node> list){
		this.list = list;
	}
}
