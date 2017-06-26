/* Main code. The process followed here is explained in the README. */

#include "../header/MAIN.h"
#include "../header/classes.h"
#include "../header/Inputs.h"

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

//----------------------------  MAIN  ----------------------------//
int main (int argc, char** argv)
{

  Parameters parameters;
  std::vector <Frame> frames;
  std::vector <Landmark> landmarks;

  // User input
  bool options_flag= false;

  parameters= GetUserInputs(argc, argv, parameters);

  // Need two clouds, original cloud to plot and cloud to work on
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
  								 cloud_original (new pcl::PointCloud<pcl::PointXYZ>);

  std::cout<<"-------------------------------------------------------------"<<std::endl;
  std::cout<<"Test Output !";

  return (0);
}