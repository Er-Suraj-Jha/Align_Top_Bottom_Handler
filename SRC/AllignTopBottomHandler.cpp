#include<vector>
#include<array>
#include<string>

class AllignTopBottom {

private:
	//function to calculate dot product of two vectors
	int dot_product(std::array<double, 3>& vector_a, std::array<double, 3>& vector_b) {
		int product = 0;
		for (int i = 0; i < 3; i++)
			product = product + vector_a[i] * vector_b[i];
		return product;
	}
	//function to calculate cross product of two vectors
	void cross_product(std::array<double, 3>& vector_a, std::array<double, 3>& vector_b, std::array<double, 3>& temp) {
		temp[0] = vector_a[1] * vector_b[2] - vector_a[2] * vector_b[1];
		temp[1] = -(vector_a[0] * vector_b[2] - vector_a[2] * vector_b[0]);
		temp[2] = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0];
	}
	//function to calculate normal of three vectors
	std::array<double, 3> CalculateNormal(std::array<double, 3>& v1, std::array<double, 3>& v2, std::array<double, 3>& v3)
	{
		std::array<double, 3> resultant_normal;
		double a1 = v2[0] - v1[0];
		double b1 = v2[1] - v1[1];
		double c1 = v2[2] - v1[2];
		double a2 = v3[0] - v1[0];
		double b2 = v3[1] - v1[1];
		double c2 = v3[2] - v1[2];

		double d0 = b1 * c2 - b2 * c1;
		double d1 = a2 * c1 - a1 * c2;
		double d2 = a1 * b2 - a2 * b1;
		double resultant_d = pow(pow(d0, 2) + pow(d1, 2) + pow(d2, 2), 0.5);


		resultant_normal[0] = d0 / resultant_d;
		resultant_normal[1] = d1 / resultant_d;
		resultant_normal[2] = d2 / resultant_d;
		return resultant_normal;
	}
	//function to find rotation matrix to align vector v1 to vector v2
	double* rotateAlign(std::array<double, 3>& v1, std::array<double, 3>& v2)
	{
		std::array<double, 3> axis;
		cross_product(v1, v2, axis);
		const float cosA = dot_product(v1,v2);
		float guard = 1.0f + cosA;
		if (guard == 0.0f)
			guard = 0.00001f;
		const float k = 1.0f / (guard);

		static double result[9];

		result[0] = (axis[0] * axis[0] * k) + cosA;
		result[1] = (axis[1] * axis[0] * k) - (axis[2]);
		result[2] = (axis[2] * axis[0] * k) + (axis[1]);
		result[3] = (axis[0] * axis[1] * k) + (axis[2]);
		if (cosA == -1)
			result[4] = (axis[1] * axis[1] * k) - cosA;
		else
			result[4] = (axis[1] * axis[1] * k) + cosA;

		result[5] = (axis[2] * axis[1] * k) - (axis[0]);
		result[6] = (axis[0] * axis[2] * k) - (axis[1]);
		result[7] = (axis[1] * axis[2] * k) + (axis[0]);
		result[8] = (axis[2] * axis[2] * k) + cosA;

		return result;
	}
	//function to find XYZ rotation from a rotation matrix
	std::array<double, 3> RotMatrix_to_XYZ_rotation(double* matrix)
	{

		std::array<double, 3> euler_angles;

		if (matrix[6] != 1 && matrix[6] != -1)
		{
			double pitch_1 = -1 * asin(matrix[6]);
			double pitch_2 = 3.141592 - pitch_1;
			double roll_1 = atan2(matrix[7] / cos(pitch_1), matrix[8] / cos(pitch_1));
			double roll_2 = atan2(matrix[7] / cos(pitch_2), matrix[8] / cos(pitch_2));
			double yaw_1 = atan2(matrix[3] / cos(pitch_1), matrix[0] / cos(pitch_1));
			double yaw_2 = atan2(matrix[3] / cos(pitch_2), matrix[0] / cos(pitch_2));


			euler_angles[1] = pitch_1;
			euler_angles[0] = roll_1;
			euler_angles[2] = yaw_1;
		}
		else
		{
			euler_angles[2] = 0;
			if (matrix[6] == -1) {
				euler_angles[1] = 3.141592 / 2;
				euler_angles[0] = euler_angles[2] + atan2(matrix[1], matrix[2]);
			}
			else
			{
				euler_angles[1] = -3.141592 / 2;
				euler_angles[0] = -1 * euler_angles[2] + atan2(-1 * matrix[1], -1 * matrix[2]);
			}

		}
		//rxyz_deg = [roll, pitch, yaw]

		return euler_angles;

	}
	//function to find rotation 
	void Align_accordingly()
	{
		double z0 = 0;
		double z1 = 0;
		double z2 = 1;

		if (Align_Type == "Top")
		{
			z0 = 0;
			z1 = 0;
			z2 = 1;
		}
		else
		{
			z0 = 0;
			z1 = 0;
			z2 = -1;
		}

		std::array<double,3> destination{z0,z1,z2};

		double* rotmatpointer = rotateAlign(normal, destination);
		XYZ_Rotation = RotMatrix_to_XYZ_rotation( rotmatpointer);
	}


public:
	void Get_Slected_Triangle_Vertices(double* selected_tri_vertices,std::string& aligntype)
	{
		Align_Type = aligntype;
		int k = 0;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				tri_vertices[0][0] = selected_tri_vertices[k];
				k++;
			}
		}

		std::array<double, 3> v1 = { tri_vertices[0][0] ,tri_vertices[0][1] ,tri_vertices[0][2] };
		std::array<double, 3> v2 = { tri_vertices[1][0] ,tri_vertices[1][1] ,tri_vertices[1][2] };
		std::array<double, 3> v3 = { tri_vertices[2][0] ,tri_vertices[2][1] ,tri_vertices[2][2] };

		normal = CalculateNormal(v1, v2, v3);

		Align_accordingly();

	}

private:
			int mat[9];
			int tri_vertices[3][3];
			std::string Align_Type;
			std::array<double, 3> normal;
			std::array<double, 3> XYZ_Rotation;
};