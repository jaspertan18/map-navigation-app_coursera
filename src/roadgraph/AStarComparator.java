package roadgraph;

import java.util.Comparator;

public class AStarComparator implements Comparator<MapNode>{

	@Override
	public int compare(MapNode o1, MapNode o2) {
		// TODO Auto-generated method stub
		if (o1.getPredictedTime() == o2.getPredictedTime())
			return 0;
		else if (o1.getPredictedTime() > o2.getPredictedTime())
			return 1;
		else
			return -1;
	}

}
