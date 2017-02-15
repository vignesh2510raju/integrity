#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "../header/user_input.h"

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

using namespace std;

int user_input(bool options_flag, 
                  int c,
                  std::string filename, 
                  float max_radius, float nd_weight,float min_cluster_distance,float min_density,float xlim,float ylim,float zlim, 
                  int min_cluster_size,int min_plane_size,int min_cylinder_size,
                  int argc)
{
        while (!options_flag)
          {
            switch (c)
            {
            /*  case 'a':
                filename= optarg;
                cout<< "Filename is: "<< optarg<< endl;
                break;
              case 'b':
                max_radius= atof(optarg);    // 0.15
                cout<< "Maximum radius is: "<< optarg<< endl;
                break;
              case 'c':
                nd_weight= atof(optarg);     // 0.1
                cout<< "Normal-distance weight is: "<< optarg<< endl;
                break;
              case 'd':
                min_cluster_size= atoi(optarg);
                cout<< "Minimum cluster size is: "<< optarg<< endl;
                break;
              case 'e':
                min_plane_size= atoi(optarg);
                cout<< "Minimum plane size is: "<< optarg<< endl;
                break;
              case 'f':
                min_cluster_distance= atof(optarg);
                cout<< "Minimum cluster distance is: "<< optarg<< endl;
                break;
              case 'g':
                min_density= atof(optarg);
                cout<< "Minimum density of a cluster is: "<< optarg<< endl;
                break;
              case 'h':
                min_cylinder_size= atoi(optarg);
                cout<< "Minimum cylinder size is: "<< optarg<< endl;
                break;
              case 'x':
                xlim= atof(optarg);
                cout<< "X limits are: [-"<< xlim<< ",+"<< xlim<< "]"<< endl;
                break;
              case 'y':
                ylim= atof(optarg);
                cout<< "Y limits are: [-"<< ylim<< ",+"<< ylim<< "]"<< endl;
                break;
              case 'z':
                zlim= atof(optarg);
                cout<< "Z limits are: [-"<< zlim<< ",+ inf]"<< endl;
                break;*/

              // If you don't want to set all the options just run the code with an option that is not here and it will take the defaults
              case '?':
                cout<< "invalid argumnet, using default options"<< endl;
                options_flag= true;           
                //filename= "000009";           // a)
                max_radius= 0.12f;            // b)
                nd_weight= 0.1f;              // c)
                min_cluster_size= 50;         // d)
                min_plane_size= 250;          // e) 
                min_cluster_distance= 0.1;    // f)
                min_density= 400;             // g)
                min_cylinder_size= 30;        // h)
                xlim= 14;                     // x)
                ylim= 14;                     // y)
                zlim= 0.5;                    // z)
                break;
              
                return max_radius, nd_weight, min_cluster_distance, min_density, xlim, ylim, zlim, min_cluster_size, min_plane_size, min_cylinder_size;

              default:
                cout<<" Default reached"<<endl;
                return (1);
            }
          }
}