// #include <iostream>
// #include <string>
// #include <unordered_map>
// #include <ros/ros.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>

// struct Position
// {
//   double x;
//   double y;
//   //double z;

//   Position(double x_, double y_) 
//   {
//       x = x_;
//       y = y_;
//       //z = z_;
//   } 

//   bool operator==(const Position &other) const
//   { return (x == other.x
//             && y == other.y);
//             //&& z == other.z);
//   }
// };

  
// class MyHashFunction { 
// public: 
  
//     // We use predfined hash functions of strings 
//     // and define our hash function as XOR of the 
//     // hash values. 
//     size_t operator()(const Position& p) const
//     { 
//         return (std::hash<double>()(p.x)) ^  
//                (std::hash<double>()(p.y)); 
//     } 
// }; 

// // namespace std {

// // template <>
// // struct hash<Position>
// // {
// //     std::size_t operator()(const Position& p) const
// //     {
// //         using std::size_t;
// //         using std::hash;
// //         // Compute individual hash values for first,
// //         // second and third and combine them using XOR
// //         // and bit shifting:

// //         return ((hash<double>()(p.x)
// //         ^ (hash<double>()(p.y) << 1)) >> 1);
// //         //^ (hash<double>()(p.z) << 1);
// //         }
// //     };
// // } 
 
// int main(int argc, char** argv){
//   ros::init(argc, argv, "my_tf_listener");

//   ros::NodeHandle node;

//     // Create an unordered_map of three strings (that map to strings)
//     std::unordered_map<std::string, std::string> u = {
//         {"RED","#FF0000"},
//         {"GREEN","#00FF00"},
//         {"BLUE","#0000FF"}
//     };
 
//     // Iterate and print keys and values of unordered_map
//     for( const auto& n : u ) {
//         std::cout << "Key:[" << n.first << "] Value:[" << n.second << "]\n";
//     }
 
//     // Add two new entries to the unordered_map
//     u["BLACK"] = "#000000";
//     u["WHITE"] = "#FFFFFF";
 
//     // Output values by key
//     std::cout << "The HEX of color RED is:[" << u["RED"] << "]\n";
//     std::cout << "The HEX of color BLACK is:[" << u["BLACK"] << "]\n";

//     double error1 = 1;
//     double error2 = 2;

    
//     //std::unordered_map<Position, Position> um;
//     std::unordered_map<Position, Position, MyHashFunction> um;

//     geometry_msgs::Point p;
//     p.x = 0;
//     p.y = 10;

//     Position p1(p.x, p.y);
//     Position p2(1, 2);
//     Position p3(0.1, 0.2);

//     Position e1(p.x, p.y);
//     Position e2(0.5, 0.001);
//     Position e3(20, 30.2);


//     um[p1] = e1;
//     // um[p2] = e2;
//     // um[p3] = e3;


//     for (auto e : um) { 
//         std::cout << "[" << e.first.x << ", " 
//              << e.first.y 
//              << "] = > " << e.second.x << ", " 
//              << e.second.y  << '\n'; 
//     } 

 
//     return 0;
// }


// CPP program to demonstrate working of unordered_map 
// for user defined data types. 
#include <bits/stdc++.h>
#include <ros/ros.h>
//using namespace std; 

struct Position { 
	double x, y; 

	Position(double x_, double y_) 
	{ 
		x = x_; 
		y = y_; 
	} 

	bool operator==(const Position& p) const
	{ 
		return x == p.x && y == p.y; 
	}

    bool operator<(const Position &o)  const {
        return x < o.x || (x == o.x && y < o.y);
    }
};


class MyHashFunction { 
public: 

	// We use predfined hash functions of strings 
	// and define our hash function as XOR of the 
	// hash values. 
	size_t operator()(const Position& p) const
	{ 
		return (std::hash<double>()(p.x)) ^ 
			(std::hash<double>()(p.y)); 
	} 
}; 

// Driver code 
int main() 
{ 
    std::unordered_map<Position, Position, MyHashFunction> um;

    Position p1(1,2);
    Position e1(3,4);
    Position e2(5,6);


    //std::pair<Position, Position> p((Position{1,2}),(Position{3,4}));
    std::pair<Position, Position> p_(p1,e1);
    um.insert(p_);

    // Values are not erased if writing a new value for the same key. Need to erase value and isert again-- //um.erase(iterator);
    //um.erase(um.find(p1));

    // std::pair<Position, Position> p(p1,e2);      
    // um.insert(p);

    //um.clear();


	// for (auto e : um) { 
	// 	std::cout << "[" << e.first.x << ", "
	// 		<< e.first.y 
	// 		<< "] = > " << e.second.x << ", "
    //         << e.second.y << '\n'; 
	// }

    //Position p_(1,2);

    if (um.find(p1) == um.end())
    {
        std::cout << "Nope\n";

    } else
    {
        auto it = um.find(p1);
        if((it->second.x != 0) || (it->second.y != 0))
        {
            std::cout << "The HEX of color RED is:[" << it->second.x << "]\n";
        }
    }
    
    
    

	return 0; 
} 
