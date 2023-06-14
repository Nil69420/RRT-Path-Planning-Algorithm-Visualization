#include <iostream>
#include <cstdio>
#include <random>
#include <include/SFML/Graphics.hpp>
#include "geometry.h"

using namespace std ;

const int WIDTH = 800 ;
const int HEIGHT = 600 ;
const int RADIUS = 5 ;
const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

const double JUMP_SIZE = (WIDTH/100.0 * HEIGHT/100.0)/1.5;
const double DISK_SIZE = JUMP_SIZE ;

int whichRRT = 3 ;

vector < Polygon > obstacles ;
Point start, stop ;
int obstacle_cnt = 1 ;

vector < Point > nodes ;
vector < int > parent, nearby ;
vector < double > cost, jumps ;
int nodeCnt = 0, goalIndex = -1 ;

vector <sf::ConvexShape> polygons ;
sf::CircleShape startingPoint, endingPoint ;
bool pathFound = 0 ;

void getInput() {
	cout << "NOTE:" << endl ;
	cout << "Height of screen: " << HEIGHT << " pixels." ;
	cout << " Width of screeen: " << WIDTH << " pixels." << endl ;
	cout << "Maximum distance by which algorithm jumps from one point to another: " << JUMP_SIZE << " units" << endl ;
	cout << "If you would like to change of any of these, please make modifications in code" << endl ;
	cout << "Please provide your inputs keeping this in mind. " << endl << endl ;

	cout << "Which type of RRT would you like to watch? 1 for RRT, 2 for RRT*, 3 for Anytime RRT" << endl ;
	cin >> whichRRT ;
	cout << "Input co-ordinates of starting and ending point respectively in this format X1 Y1 X2 Y2" << endl ;
	cin >> start.x >> start.y >> stop.x >> stop.y ;
	cout << "How many obstacles?" << endl ;
	cin >> obstacle_cnt ;

	obstacles.resize(obstacle_cnt);
	int pnts = 0 ; Point pnt ;
	vector < Point > poly ;

	for(int i = 0; i < obstacle_cnt; i++)
        {
		poly.clear();
		cout << "How many points in " << i+1 << "th polygon?" << endl ;
		cin >> pnts ;
		poly.resize(pnts);

		cout << "Input co-ordinates of " << i+1 << "th polygon in clockwise order" << endl ;
		for(int j = 0; j < pnts; j++) {
			cin >> pnt.x >> pnt.y ;
			obstacles[i].addPoint(pnt);
		}
	}
}


void prepareInput()
{
	startingPoint.setRadius(RADIUS); endingPoint.setRadius(RADIUS);
    startingPoint.setFillColor(sf::Color(208, 0, 240)); endingPoint.setFillColor(sf::Color::Blue);
    startingPoint.setPosition(start.x, start.y); endingPoint.setPosition(stop.x, stop.y);
    startingPoint.setOrigin(RADIUS/2, RADIUS/2); endingPoint.setOrigin(RADIUS/2, RADIUS/2);


	polygons.resize(obstacle_cnt);
	for(int i = 0; i < obstacle_cnt; i++)
        {
		polygons[i].setPointCount(obstacles[i].pointCnt);
		polygons[i].setFillColor(sf::Color(89, 87, 98));
		for(int j = 0; j < obstacles[i].pointCnt; j++)
			polygons[i].setPoint(j, sf::Vector2f(obstacles[i].points[j].x, obstacles[i].points[j].y));
        }
}

void draw(sf::RenderWindow& window)
{
	sf::Vertex line[2]; sf::CircleShape nodeCircle;

	for(auto& node: nodes)
        {
		nodeCircle.setRadius(RADIUS/2.5);
		nodeCircle.setOrigin(RADIUS/2.5, RADIUS/2.5);
		nodeCircle.setFillColor(sf::Color(0, 255, 171)); nodeCircle.setPosition(node.x, node.y);
		window.draw(nodeCircle);
	}

	for(auto& poly : polygons) window.draw(poly);

	for(int i = (int)nodes.size() - 1; i; i--)
        {
		Point par = nodes[parent[i]] ;
		line[0] = sf::Vertex(sf::Vector2f(par.x, par.y));
		line[1] = sf::Vertex(sf::Vector2f(nodes[i].x, nodes[i].y));
		window.draw(line, 2, sf::Lines);
        }

	window.draw(startingPoint); window.draw(endingPoint);

	if(pathFound)
        {
		int node = goalIndex;
		while(parent[node] != node)
            {
			int par = parent[node];
			line[0] = sf::Vertex(sf::Vector2f(nodes[par].x, nodes[par].y));
			line[1] = sf::Vertex(sf::Vector2f(nodes[node].x, nodes[node].y));
			line[0].color = line[1].color = sf::Color::Red;
			window.draw(line, 2, sf::Lines);
			node = par ;
            }
	}
}

template <typename T>
T randomCoordinate(T low, T high)
{
    random_device random_device;
    mt19937 engine{random_device()};
    uniform_real_distribution<double> dist(low, high);
    return dist(engine);
}

bool isEdgeObstacleFree(Point a, Point b)
{
    for(auto& poly: obstacles)
        if(lineSegmentIntersectsPolygon(a, b, poly))
        	return false ;
    return true ;
}
Point pickRandomPoint()
{
    double random_sample = randomCoordinate(0.0, 1.0);
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) return stop + Point(RADIUS, RADIUS) ;
	return Point(randomCoordinate(0, WIDTH), randomCoordinate(0, HEIGHT));
}

void checkDestinationReached()
{
	sf::Vector2f position = endingPoint.getPosition();
	if(checkCollision(nodes[parent[nodeCnt - 1]], nodes.back(), Point(position.x, position.y), RADIUS))
        {
		pathFound = 1 ;
		goalIndex = nodeCnt - 1;
		cout << "Reached!! With a distance of " << cost.back() << " units. " << endl << endl ;
        }
}

void insertNodesInPath(int rootIndex, Point& q)
{
	Point p = nodes[rootIndex] ;
	if(!isEdgeObstacleFree(p, q)) return ;
	while(!(p == q))
        {
		Point nxt = p.steer(q, JUMP_SIZE);
		nodes.push_back(nxt);
		parent.push_back(rootIndex);
		cost.push_back(cost[rootIndex] + distance(p, nxt));
		rootIndex = nodeCnt++ ;
		p = nxt ;
        }
}

void rewire()
{
	int lastInserted = nodeCnt - 1 ;
	for(auto nodeIndex: nearby)
        {
		int par = lastInserted, cur = nodeIndex;
		while( ((cost[par] + distance(nodes[par], nodes[cur])) - cost[cur]) <= EPS)
		{
			int oldParent = parent[cur] ;
			parent[cur] = par; cost[cur] = cost[par] + distance(nodes[par], nodes[cur]);
			par = cur, cur = oldParent;
		}
	}
}

void RRT()
{
	Point newPoint, nearestPoint, nextPoint ; bool updated = false ; int cnt = 0 ;
	int nearestIndex = 0 ; double minCost = INF; nearby.clear(); jumps.resize(nodeCnt);

	while(!updated) {
		newPoint = pickRandomPoint();
		nearestPoint = *nodes.begin(); nearestIndex = 0;
		for(int i = 0; i < nodeCnt; i++)
            {
			if(pathFound and randomCoordinate(0.0, 1.0) < 0.25)
				cost[i] = cost[parent[i]] + distance(nodes[parent[i]], nodes[i]);

			jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE ;
			auto pnt = nodes[i] ;
			if((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS and isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps[i])))
				nearestPoint = pnt, nearestIndex = i ;
            }
		nextPoint = stepNear(nearestPoint, newPoint, jumps[nearestIndex]);
		if(!isEdgeObstacleFree(nearestPoint, nextPoint)) continue ;

		if( (whichRRT == 1) or (!pathFound and whichRRT == 3))
            {
			updated = true ;
			nodes.push_back(nextPoint); nodeCnt++;
			parent.push_back(nearestIndex);
			cost.push_back(cost[nearestIndex] + distance(nearestPoint, nextPoint));
			if(!pathFound) checkDestinationReached();
			continue ;
            }
		for(int i = 0; i < nodeCnt; i++)
			if((nodes[i].distance(nextPoint) - DISK_SIZE) <= EPS and isEdgeObstacleFree(nodes[i], nextPoint))
				nearby.push_back(i);
		int par = nearestIndex; minCost = cost[par] + distance(nodes[par], nextPoint);
		for(auto nodeIndex: nearby)
		{
			if( ( (cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint)) - minCost) <= EPS)
				minCost = cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint), par = nodeIndex;
		}

		parent.push_back(par); cost.push_back(minCost);
		nodes.push_back(nextPoint); nodeCnt++;
		updated = true ;
		if(!pathFound) checkDestinationReached();
		rewire();
	}
}

int main()
{
	getInput(); prepareInput();
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Basic Anytime RRT");

	nodeCnt = 1; nodes.push_back(start); int iterations = 0 ;
	parent.push_back(0); cost.push_back(0);
    sf::Time delayTime = sf::milliseconds(5);

    cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl ;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
            	window.close();
            	return 0; exit(0);
            }
        }
        RRT(); iterations++;

		if(iterations % 500 == 0)
            {
			cout << "Iterations: " << iterations << endl ;
			if(!pathFound) cout << "Not reached yet :( " << endl ;
			else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl ;
			cout << endl ;
            }

		//sf::sleep(delayTime);
		window.clear();
		draw(window);
        window.display();
    }
}

