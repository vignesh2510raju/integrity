//
//  Inputs.cpp
//  UserInput
//
//  Created by Guillermo Duenas Arana on 2/11/17.
//  Copyright © 2017 Guille. All rights reserved.
//

/* This function let's the user change the parameters without compiling the code 
every time. The options are selected starting with "--" + parameter + " " + value.
For example, if we want the code to read 30 pcd files, we run:
./main --numFrames 30
The values not specified here are initialized by the default constructor of the 
struct "Parameters" which is defined in "classes.cpp". */

#include "../header/Inputs.h"

using namespace std;

Parameters GetUserInputs(int argc, char* argv[], Parameters p)
{

    static int verbose_flag;
    int c;

    while(1)
    {
        static struct option long_options[] =
        {
          /* These options set a flag. */
          {"verbose", no_argument,       &verbose_flag, 1},
          {"brief",   no_argument,       &verbose_flag, 0},
          /* These options don’t set a flag.
             We distinguish them by their indices. */
          {"numFrames",                       required_argument,       0, 'a'},
          {"min_cluster_size",                required_argument,       0, 'b'},
          {"min_plane_size",                  required_argument,       0, 'c'},
          {"min_density",                     required_argument,       0, 'd'},
          {"xlim",                            required_argument,       0, 'e'},
          {"ylim",                            required_argument,       0, 'f'},
          {"zlim",                            required_argument,       0, 'g'},
          {"cylinder_normal_distance_weight", required_argument,       0, 'h'},
          {"cylinder_max_radius",             required_argument,       0, 'i'},
          {"options_flag",                    required_argument,       0, 'j'},
          {"SWITH_VOXEL_GRID",                required_argument,       0, 'k'},
          {"SWITH_WRITE_CLUSTERS",            required_argument,       0, 'l'},
          {"plane_distance_threshold",        required_argument,       0, 'm'},
          {"plane_normal_distance_weight",    required_argument,       0, 'n'},
          {"plane_RANSAC_max_iter",           required_argument,       0, 'o'},
          {"normal_radius_search",            required_argument,       0, 'p'},
          {"min_cylinder_size",               required_argument,       0, 'q'},
          {"min_association_distance",        required_argument,       0, 'r'},
          {"rlim",                            required_argument,       0, 's'},
          {0, 0, 0, 0}
        };
        
        /* getopt_long stores the option index here. */
        int option_index = 0;

        c = getopt_long (argc, argv, "a:b:c:d:f:",long_options, &option_index);

        /* Detect the end of the options. */
        if (c == -1)
            break;

        switch (c)
        {
            case 0:
              /* If this option set a flag, do nothing else now. */
              if (long_options[option_index].flag != 0)
                break;

              printf ("option %s", long_options[option_index].name);
              if (optarg)
                printf (" with arg %s", optarg);
              printf ("\n");
              break;

            case 'a':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.numFrames= atoi(optarg);
              break;

            case 'b':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.min_cluster_size= atoi(optarg);
              break;

            case 'c':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.min_plane_size= atoi(optarg);
              break;

            case 'd':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.min_density= atof(optarg);
              break;

            case 'e':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.xlim= atof(optarg);
              break;

            case 'f':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.ylim= atof(optarg);
              break;

            case 'g':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.zlim= atof(optarg);
              break;

            case 'h':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.cylinder_normal_distance_weight= atof(optarg);
              break;

            case 'i':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.cylinder_max_radius= atof(optarg);
              break;

            case 'j':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.options_flag= (bool) atoi(optarg);
              break;

            case 'k':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.SWITH_VOXEL_GRID= (bool) atoi(optarg);
              break;

            case 'l':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.SWITH_WRITE_CLUSTERS= (bool) atoi(optarg);
              break;

            case 'm':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.plane_distance_threshold= atof(optarg);
              break;

            case 'n':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.plane_normal_distance_weight= atof(optarg);
              break;

            case 'o':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.plane_RANSAC_max_iter= atoi(optarg);
              break;

            case 'p':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.normal_radius_search= atof(optarg);
              break;

            case 'q':
              printf ("option %s with value `%s'\n", long_options[option_index].name, optarg);
              p.min_cylinder_size= atoi(optarg);
              break;

            case '?':
              /* getopt_long already printed an error message. */
              break;

            default:
              abort ();
        }

    }
    cout<< "returning the user parameters"<< endl;
    return (p);

}

//funcion that show the help information
void showhelpinfo(const char *s)
{
    std::cout<<"Usage:   "<<s<<" [-option] [argument]"<<std::endl;
    std::cout<<"option:  "<<"-h  show help information"<<std::endl;
    std::cout<<"         "<<"-u username"<<std::endl;
    std::cout<<"         "<<"-p  password"<<std::endl;
    std::cout<<"         "<<"-s  save the password: 0(save password) 1(forget password)"<<std::endl;
    std::cout<<"         "<<"-v  show version infomation"<<std::endl;
    std::cout<<"example: "<<s<<" -uusername -ppassword -s1"<<std::endl;
}
