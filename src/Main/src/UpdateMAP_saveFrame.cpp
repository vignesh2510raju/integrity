
#include "../header/UpdateMAP_saveFrame.h"

using namespace std;

void UpdateMAP_saveFrame(std::vector<Cylinder> cylinders, 
						 std::vector<Frame> &frames, 
						 std::vector<Landmark> &landmarks, 
						 Eigen::Matrix4d T,
						 Parameters p)
{
	int index;

	Landmark landmark;
	landmark.rep= 1;

	Frame actual_frame;
	actual_frame.numFeatures= cylinders.size();

	float minDistance, distance;

	Eigen::Vector2d carPose2x1;
	carPose2x1<< T(0,3),
				 T(2,3);
	cout<<"car pose: \n"<< carPose2x1<< endl;

	actual_frame.numDetected= 0;
	actual_frame.numExpected= 0;

	// Get the expected number of landmark
	for (int i = 0; i < landmarks.size(); ++i)
	{
		if ( (carPose2x1 - landmarks[i].pose).squaredNorm() < p.rlim^2 )
			actual_frame.numExpected++;
	}

	// Loop through features in current scan
	for (int i = 0; i < actual_frame.numFeatures; ++i)
	{
		// Consider the feature non-associated initially
		actual_frame.association.push_back(-1);
		minDistance= p.min_association_distance + 1;
		distance= minDistance;

		// Loop through landmarks in the map
		for (int l = 0; l < landmarks.size(); ++l)
		{


			distance= (cylinders[i].pose - landmarks[l].pose).squaredNorm();
			if (distance < minDistance)
			{
				minDistance= distance;
				index= l; 
			}
		}

		if (minDistance < p.min_association_distance)
		{
			actual_frame.numDetected++;
			landmarks[index].rep++;
			actual_frame.association[i]= index; // if not it was set to -1
		}
		

		cout<< "Cylinder "<< i<< "associated with landmark "
			<< actual_frame.association[i]<< endl;
	}
	if (actual_frame.numExpected == 0)
	{
	 	actual_frame.repRate= 1;
	} else {
		actual_frame.repRate= (double) actual_frame.numDetected / actual_frame.numExpected;
	}
	
	// Store the new frame
	frames.push_back(actual_frame);



	// Update the map with the non-associated
	for (int i = 0; i < actual_frame.numFeatures; ++i)
	{
		if (actual_frame.association[i] == -1)
		{
			landmark.pose= cylinders[i].pose;
			landmarks.push_back(landmark);
		}
	}

	cout<< "Size of the map: "<< landmarks.size()<< endl;
}


































