using System.Collections;
using System.Collections.Generic;
using System;
using System.Globalization;
using UnityEngine;
using System.Diagnostics;

public class Pathfinding : MonoBehaviour {

	public Transform seeker, target;
	Grid grid;
	Node startNode;
	Node targetNode;
	public long AStarTime1, AStarTime2, BFSTime, DFSTime, UCSTime;
	public long AStarCount1 = 0, AStarCount2 = 0, BFSCount = 0, DFSCount = 0, UCSCount = 0;
	public long AStarFringeLength1 = 1, AStarFringeLength2 = 1, BFSFringeLength = 1, DFSFringeLength = 1, UCSFringeLength = 1;

	void Awake()
	{
		grid = GetComponent<Grid>();
		
	}

	void Update()
	{
		startNode = grid.NodeFromWorldPoint(seeker.position);
		targetNode = grid.NodeFromWorldPoint(target.position);

		BFS();
		print("BFS ==> " + BFSTime + " ms");
		print("BFS ==> " + BFSCount);
		print("BFS ==> " + BFSFringeLength);

		DFS();
		print("DFS ==> " + DFSTime + " ms");
		print("DFS ==> " + DFSCount);
		print("DFS ==> " + DFSFringeLength);

		UCS();
		print("UCS ==> " + UCSTime + " ms");
		print("UCS ==> " + UCSCount);
		print("UCS ==> " + UCSFringeLength);

		Astar1();
		print("Astar1 ==> " + AStarTime1 + " ms");
		print("Astar1 ==> " + AStarCount1);
		print("Astar1 ==> " + AStarFringeLength1);

		Astar2();
		print("Astar2 ==> " + AStarTime2 + " ms");
		print("Astar2 ==> " + AStarCount2);
		print("Astar2 ==> " + AStarFringeLength2);
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
				RetracePath(startNode, targetNode, 1);
				return;
			}

			BFSCount++;
			closedSet.Add(currentNode);

			List<Node> neighbours = new List<Node>();
			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (!neighbour.walkable)
					continue;
				else {
					neighbour.parent = currentNode;
					fringe.Enqueue(neighbour);
				}
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
				RetracePath(startNode, targetNode, 2);
				return;
			}

			DFSCount++;
			closedSet.Add(currentNode);

			List<Node> neighbours = new List<Node>();
			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (!neighbour.walkable)
					continue;
				else
				{
					neighbour.parent = currentNode;
					fringe.Push(neighbour);
				}
			}

			if (fringe.Count > DFSFringeLength)
			{
				DFSFringeLength = fringe.Count;
			}

		}
	}

	void UCS()	{

		Stopwatch stopwatch = new Stopwatch();
		stopwatch.Start();

		Heap<Node> fringe = new Heap<Node>(grid.MaxSize);
		startNode.gCost = 0;
		startNode.hCost = 0;
		fringe.Add(startNode);

		HashSet<Node> closedSet = new HashSet<Node>();
		Node currentNode;

		while (fringe.Count > 0) {
			currentNode = fringe.RemoveFirst();

			if (closedSet.Contains(currentNode))
				continue;

			if (currentNode == targetNode)
			{
				stopwatch.Stop();
				UCSTime = stopwatch.ElapsedMilliseconds;
				RetracePath(startNode, targetNode, 3);
				return;
			}

			UCSCount++;
			closedSet.Add(currentNode);

			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (!neighbour.walkable) {
					continue;
				}

				neighbour.gCost = currentNode.gCost + GetDistance(currentNode, neighbour);
				neighbour.hCost = 0;
				fringe.Add(neighbour);

				neighbour.parent = currentNode;
			}
			if (fringe.Count > UCSFringeLength)
			{
				UCSFringeLength = fringe.Count;
			}
		}
	}


	void Astar1() {
		Stopwatch stopwatch = new Stopwatch();
		stopwatch.Start();

		Heap<Node> openSet = new Heap<Node>(grid.MaxSize);
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node currentNode = openSet.RemoveFirst();
			AStarCount1++;
			closedSet.Add(currentNode);

			if (currentNode == targetNode)
			{
				RetracePath(startNode, targetNode, 4);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newMovementCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour);
				if (newMovementCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newMovementCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = currentNode;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
					else
					{
						openSet.UpdateItem(neighbour);
					}
				}
			}
			if (openSet.Count > AStarFringeLength1) {
				AStarFringeLength1 = openSet.Count;
			}
		}
	}

	void Astar2()
	{
		Stopwatch stopwatch = new Stopwatch();
		stopwatch.Start();

		Heap<Node> openSet = new Heap<Node>(grid.MaxSize);
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node currentNode = openSet.RemoveFirst();
			AStarCount2++;
			closedSet.Add(currentNode);

			if (currentNode == targetNode)
			{
				RetracePath(startNode, targetNode, 5);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newMovementCostToNeighbour = currentNode.gCost + GetDistance2(currentNode, neighbour);
				if (newMovementCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newMovementCostToNeighbour;
					neighbour.hCost = GetDistance2(neighbour, targetNode);
					neighbour.parent = currentNode;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
					else
					{
						openSet.UpdateItem(neighbour);
					}
				}
			}
			if (openSet.Count > AStarFringeLength2)
			{
				AStarFringeLength2 = openSet.Count;
			}
		}
	}

	void RetracePath(Node startNode, Node endNode, int algo)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		if (algo == 1)
		{
			grid.BFSpath = path;
		}
		else if (algo == 2)
		{
			grid.DFSpath = path;
		}
		else if (algo == 3)
		{
			grid.UCSpath = path;
		}
		else if (algo == 4)
		{
			grid.AStarpath1 = path;
		}
		else
		{
			grid.AStarpath2 = path;
		}

	}

	int GetDistance(Node nodeA, Node nodeB)
	{
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
		
		if (dstX > dstY)
			return 14 * dstY + 10 * (dstX - dstY);
		return 14 * dstX + 10 * (dstY - dstX);
	}

	int GetDistance2(Node nodeA, Node nodeB)
	{
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
		
		return (int)Math.Sqrt(Math.Pow(10 * dstX, 2) + Math.Pow(10 * dstY, 2));
		
	}
}
