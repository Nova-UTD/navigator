---
layout: default
date:   2021-09-2
categories: recruitment
author: "Will Heitman"
---

```
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
using namespace std;

class Point {
    public:
        Point() {
            m_x = 0;
            m_y = 0;
            m_z = 0;
        }
        Point(float x, float y, float z) {
            m_x = x;
            m_y = y;
            m_z = z;
        }
        float getX() { return m_x; }
        float getY() { return m_y; }
        float getZ() { return m_z; }
        void setX(float x) { m_x = x; }
        void setY(float y) { m_y = y; }
        void setZ(float z) { m_z = z; }

    private:
        float m_x;
        float m_y;
        float m_z;
};

class Octree {

    public:
        Octree() {
            m_max_point = Point();
            m_min_point = Point();
        }

        Octree(Point min_point, Point max_point) {
            m_min_point = min_point; // Init to (0,0,0)
            m_max_point = max_point;
        }

        Octree(Point max_point, Point min_point, vector<Point> points) {
            m_min_point = min_point;
            m_max_point = max_point;
            m_points = points;
        }

    private:
    Point m_max_point;
    Point m_min_point;

    

    vector<Point> m_points;
    // m_children = new Octree[8];
    char m_active_nodes = 0;
    // Octree _parent; [necessary???]
};

vector<Point> read_points() {
    std::ifstream fin("input.csv");
    string line;
    vector<Point> points;
    
    if (fin.is_open()) {
        getline( fin, line ); //skip past the header, which has NaNs
        while ( getline (fin,line) ) {
            vector<string> column_strings;
            vector<float> column_vals;
            boost::split(column_strings, line, boost::is_any_of(","));
            // cout << column_strings.size();
            for (auto string : column_strings) {
                try {
                    // cout << stof(string);
                    column_vals.push_back(stof(string));
                    if (column_vals.size() > 2 ) {
                        points.push_back(Point(
                            column_vals.at(0),
                            column_vals.at(1),
                            column_vals.at(2)
                        ));
                    }

                } catch (invalid_argument e) {
                    cout << "ERR " << e.what() << endl;
                }
                // cout << string << stof(string) << endl;
                // column_vals.push_back(stof(string));
            }

        }
        fin.close();
    } else {
        cout << "Unable to open file.";
    }

    return points;
}

// points_in: Original points
// prob: Likelihood that a random point is kept, from 0.0 to 1.0 (certain)
vector<Point> downsample_randomly(vector<Point> points_in, float prob) {
    
    vector<Point> points_out;

    // Iterate through all points
    for( auto point : points_in ) {
        //random_float is 0.0 to 1.0
        float random_float = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        
        // Determine if point should be kept
        if (random_float <= prob) {
            points_out.push_back(point);
        } // else do nothing, skip it
    }
    
    return points_out;

}

void output_csv(vector<Point> points, string filename) {

    ofstream file(filename);

    if (file.is_open()) {
        // Write our header from the input file
        file << "x,y,z" << endl;
        for( auto point : points ) {
            file << point.getX() << "," << point.getY() << "," << point.getZ() << endl;
        }
        file.close();
    }

}

int main() {
    // std::cout << "Hello, world!";
    vector<Point> original_points = read_points();
    vector<Point> downsampled_points = downsample_randomly(original_points, 0.3);

    output_csv(downsampled_points, "output.csv");

    cout << "Kept "<<(static_cast <float> (downsampled_points.size()) / static_cast <float> (original_points.size()))*100 <<"\% of points" <<endl;
}
```
