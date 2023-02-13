using System.Collections;
using System.Collections.Generic;
using System;
using System.Globalization;
using UnityEngine;
using System.Diagnostics;

public class Astar1 : MonoBehaviour {

	public Transform seeker, target;
	Grid grid;
	Node startNode;
	Node targetNode;
	public long AStarTime, AStarAltTime, BFSTime, DFSTime, UCSTime;
	public long AStarCount = 0, AStarAltCount = 0, BFSCount = 0, DFSCount = 0, UCSCount = 0;
	public long AStarFringeLength = 1, AStarAltFringeLength = 1, BFSFringeLength = 1, DFSFringeLength = 1, UCSFringeLength = 1;

	void Awake()
	{
		grid = GetComponent<Grid>();
		startNode = grid.NodeFromWorldPoint(seeker.position);
		targetNode = grid.NodeFromWorldPoint(target.position);

		BFS();
		print("DFS ==> " + DFSTime + " ms");
	}

	void Update()
	{
		
	}

	public void BFS()
	{
		Stopwatch stopwatch = new Stopwatch();
		stopwatch.Start();

		HashSet<Node> closedSet = new HashSet<Node>();
		Queue<Node> fringe = new Queue<Node>();

		fringe.Enqueue(startNode);
		Node currentNode;
		while (fringe.Count > 0) {
			currentNode = fringe.Dequeue();

			if (closedSet.Contains(currentNode))
				continue;

			if (currentNode == targetNode)
			{
				stopwatch.Stop();
				BFSTime = stopwatch.ElapsedMilliseconds;
				RetracePath(startNode, targetNode);
				return;
			}

			BFSCount++;
			closedSet.Add(currentNode);

			List<Node> neighbours = new List<Node>();
			foreach (Node n in grid.GetNeighbours(currentNode))
			{
				if (n.walkable)
					neighbours.Add(n);
			}

			foreach (Node neighbour in neighbours)
			{
				neighbour.parent = currentNode;
				fringe.Enqueue(neighbour);
			}

			if (fringe.Count > BFSFringeLength)
			{
				BFSFringeLength = fringe.Count;
			}

		} 
	}

	public void DFS()
	{
		Stopwatch stopwatch = new Stopwatch();
		stopwatch.Start();

		HashSet<Node> closedSet = new HashSet<Node>();
		Stack<Node> fringe = new Stack<Node>();

		fringe.Push(startNode);
		Node currentNode;
		while (fringe.Count > 0)
		{
			currentNode = fringe.Pop();

			if (closedSet.Contains(currentNode))
				continue;

			if (currentNode == targetNode)
			{
				stopwatch.Stop();
				BFSTime = stopwatch.ElapsedMilliseconds;
				RetracePath(startNode, targetNode);
				return;
			}

			DFSCount++;
			closedSet.Add(currentNode);

			List<Node> neighbours = new List<Node>();
			foreach (Node n in grid.GetNeighbours(currentNode))
			{
				if (n.walkable)
					neighbours.Add(n);
			}

			foreach (Node neighbour in neighbours)
			{
				neighbour.parent = currentNode;
				fringe.Push(neighbour);
			}

			if (fringe.Count > DFSFringeLength)
			{
				DFSFringeLength = fringe.Count;
			}

		}
	}

	void Astar() {	
		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePath(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				//count++;
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void RetracePath(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.path = path;

	}

	int GetDistance(Node nodeA, Node nodeB)
	{
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14 * dstY + 10 * (dstX - dstY);
		return 14 * dstX + 10 * (dstY - dstX);
	}
}
