// Depth2PointCloud.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "Depth2PointCloud.h"
#include <string>

cv::Mat depthmap2pointmap(cv::Mat& depth, cv::Mat& KK_depth) {
	cv::Mat1i U_temp, V_temp;
	//std::cout << "d0\n";
	int height = depth.rows;
	//std::cout << "d1\n";
	int width = depth.cols;

	//clock_t begin = clock();
	//std::cout << "d2\n";
	meshgridTest(cv::Range(0.5, width - 0.5), cv::Range(0.5, height - 0.5), U_temp, V_temp);
	//std::cout << "d3\n";
	cv::Mat U, V;
	//cv::Mat Z = depth;
	cv::Mat Z;
	depth.convertTo(Z, CV_32FC1);
	//cv::flip(depth, depth, 1);
	U_temp.convertTo(U, CV_32FC1);
	V_temp.convertTo(V, CV_32FC1);
	//cout << depth.type() << endl;
	//std::cout << U.type() << " - ????\n";
	//std::cout << "d4\n";
	cv::Mat Z_U = Z.mul(U);
	//std::cout << "d5\n";
	cv::Mat Z_V = Z.mul(V);
	//std::cout << "d6\n";
	//cv::Mat KK_t = cam.KK_depth;
	//std::cout << "d7\n";
	cv::Mat output = cv::Mat(Z.rows, Z.cols, CV_32FC3);
	//cv::Mat temp = cv::Mat(1, 3, CV_16UC1);
	//std::cout << cam.id << " " << KK_t.cols << " - cols\n";
	//std::cout << "d8\n";
	cv::Mat KK_inv = KK_depth.inv();

	//std::cout << KK_depth.at<double>(0, 0) << " " << KK_depth.at<double>(0, 1) << " " << KK_depth.at<double>(0, 2) << "\n"  << KK_depth.at<double>(1, 0) << " " << KK_depth.at<double>(1, 1) << " " << KK_depth.at<double>(1, 2) << "\n" << KK_depth.at<double>(2, 0) << " " << KK_depth.at<double>(2, 1) << " " << KK_depth.at<double>(2, 2) << "\n\n";

	//std::cout << KK_inv.at<double>(0, 0) << " " << KK_inv.at<double>(0, 1) << " " << KK_inv.at<double>(0, 2) << "\n" << KK_inv.at<double>(1, 0) << " " << KK_inv.at<double>(1, 1) << " " << KK_inv.at<double>(1, 2) << "\n" << KK_inv.at<double>(2, 0) << " " << KK_inv.at<double>(2, 1) << " " << KK_inv.at<double>(2, 2) << "\n\n";

	//std::cout << "KK data type: " << KK_inv.type() << "\n";
	KK_inv.convertTo(KK_inv, CV_32FC1);

	//std::cout << KK_inv.at<float>(0, 0) << " " << KK_inv.at<float>(0, 1) << " " << KK_inv.at<float>(0, 2) << "\n" << KK_inv.at<float>(1, 0) << " " << KK_inv.at<float>(1, 1) << " " << KK_inv.at<float>(1, 2) << "\n" << KK_inv.at<float>(2, 0) << " " << KK_inv.at<float>(2, 1) << " " << KK_inv.at<float>(2, 2) << "\n";

	float *input_KK_inv = (float*)(KK_inv.data);

	float temp_v1;
	float temp_v2;
	float temp_v3;

	Eigen::Vector3f a;
	Eigen::Vector3f pointWorld;

	std::cout << "depth1: " << Z.size() << "\n";

	//std::cout << "d9\n";
//#pragma omp parallel for
	for (int i = 0; i < Z.rows; i++)
	{
//#pragma omp parallel for
		for (int j = 0; j < Z.cols; j++)
		{
			temp_v1 = Z_U.at<float>(i, j);
			temp_v2 = Z_V.at<float>(i, j);
			temp_v3 = Z.at<float>(i, j);

			//float a1 = temp_v1 * input_KK_t_inv[0] + temp_v2 * input_KK_t_inv[3] + temp_v3 * input_KK_t_inv[6];
			//float a2 = temp_v1 * input_KK_t_inv[1] + temp_v2 * input_KK_t_inv[4] + temp_v3 * input_KK_t_inv[7];
			//float a3 = temp_v1 * input_KK_t_inv[2] + temp_v2 * input_KK_t_inv[5] + temp_v3 * input_KK_t_inv[8];

			//a[0] = 0.001 * (temp_v1 * input_KK_t_inv[0] + temp_v2 * input_KK_t_inv[3] + temp_v3 * input_KK_t_inv[6]);
			//a[1] = 0.001 * (temp_v1 * input_KK_t_inv[1] + temp_v2 * input_KK_t_inv[4] + temp_v3 * input_KK_t_inv[7]);
			//a[2] = 0.001 * (temp_v1 * input_KK_t_inv[2] + temp_v2 * input_KK_t_inv[5] + temp_v3 * input_KK_t_inv[8]);

			a[0] = 0.001 * (temp_v1 * input_KK_inv[0] + temp_v2 * input_KK_inv[1] + temp_v3 * input_KK_inv[2]);
			a[1] = 0.001 * (temp_v1 * input_KK_inv[3] + temp_v2 * input_KK_inv[4] + temp_v3 * input_KK_inv[5]);
			a[2] = 0.001 * (temp_v1 * input_KK_inv[6] + temp_v2 * input_KK_inv[7] + temp_v3 * input_KK_inv[8]);

			//a = 0.001 * (temp_v1 * input_KK_t_inv[2] + temp_v2 * input_KK_t_inv[5] + temp_v3 * input_KK_t_inv[8]);

			//a[0] = (temp_v1 * input_KK_t_inv[0] + temp_v2 * input_KK_t_inv[3] + temp_v3 * input_KK_t_inv[6]);
			//a[1] = (temp_v1 * input_KK_t_inv[1] + temp_v2 * input_KK_t_inv[4] + temp_v3 * input_KK_t_inv[7]);
			//a[2] = (temp_v1 * input_KK_t_inv[2] + temp_v2 * input_KK_t_inv[5] + temp_v3 * input_KK_t_inv[8]);

			//pointWorld = cam.Rt_depth_to_color.block<3, 3>(0, 0) * (a + cam.Rt_depth_to_color.block<3, 1>(0, 3));
			//pointWorld = cam.Rt_curr_to_world.block<3, 3>(0, 0) * a + cam.Rt_curr_to_world.block<3, 1>(0, 3);

			//if (pointWorld[2] < -0.4 && pointWorld[2] > -2)
			if (a[2] > 0.4 && a[2] < 4)
			{
				// cv::Mat result = temp*KK_t.inv();
				output.at<cv::Vec3f>(i, j)[0] = a[0];	//0.001 * (temp_v1 * input_KK_t_inv[0] + temp_v2 * input_KK_t_inv[3] + temp_v3 * input_KK_t_inv[6]);// result.at<Vec3f>(0, 0)[0];
				output.at<cv::Vec3f>(i, j)[1] = a[1];	//0.001 * (temp_v1 * input_KK_t_inv[1] + temp_v2 * input_KK_t_inv[4] + temp_v3 * input_KK_t_inv[7]);// result.at<Vec3f>(0, 0)[1];
				output.at<cv::Vec3f>(i, j)[2] = a[2];	//a;// result.at<Vec3f>(0, 0)[2];
				// std::cout << output.at<cv::Vec3f>(i, j)[2] << "\n";
				//std::cout << pointWorld[0] << " " << pointWorld[1] << " " << pointWorld[2] << "\n";
			}
			//else
			//{
				//output.at<cv::Vec3f>(i, j)[0] = -0.0000000001;
				//output.at<cv::Vec3f>(i, j)[1] = -0.0000000001;
				//output.at<cv::Vec3f>(i, j)[2] = -0.0000000001;
			//}
		}
	}
	
	//	 clock_t end1 = clock();
	//	 double elapsed_secs = double(end1 - begin) / CLOCKS_PER_SEC;
	//	 cout << "time elapsed1: " << elapsed_secs << endl;
	
	//Sleep(3000);
	

	return output;
}

void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv,
	cv::Mat1i &X, cv::Mat1i &Y)
{
	cv::repeat(xgv.reshape(1, 1), ygv.total(), 1, X);
	cv::repeat(ygv.reshape(1, 1).t(), 1, xgv.total(), Y);
}

void meshgridG(const cv::Mat &xgv, const cv::Mat &ygv,
	cv::Mat &X, cv::Mat &Y)
{
	cv::repeat(xgv.reshape(1, 1), ygv.total(), 1, X);
	cv::repeat(ygv.reshape(1, 1).t(), 1, xgv.total(), Y);
}


static void meshgridTest(const cv::Range &xgv, const cv::Range &ygv,
	cv::Mat1i &X, cv::Mat1i &Y)
{
	std::vector<int> t_x, t_y;
	for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);
	for (int i = ygv.start; i <= ygv.end; i++) t_y.push_back(i);
	meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}

static void meshgridTestG(const cv::Range &xgv, const cv::Range &ygv, cv::Mat &X, cv::Mat &Y)
{
	std::vector<int> t_x, t_y;
	for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);
	for (int i = ygv.start; i <= ygv.end; i++) t_y.push_back(i);
	meshgridG(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}


/*
static void meshgridTest(const cv::Range &xgv, const cv::Range &ygv,
	cv::Mat1i &X, cv::Mat1i &Y)
{
	std::vector<int> t_x, t_y;
	for (int i = xgv.start; i <= xgv.end; i+=2) t_x.push_back(i);
	for (int i = ygv.start; i <= ygv.end; i+=2) t_y.push_back(i);
	meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}

static void meshgridTestG(const cv::Range &xgv, const cv::Range &ygv, cv::Mat &X, cv::Mat &Y)
{
	std::vector<int> t_x, t_y;
	for (int i = xgv.start; i <= xgv.end; i+=2) t_x.push_back(i);
	for (int i = ygv.start; i <= ygv.end; i+=2) t_y.push_back(i);
	meshgridG(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}
*/

void  selectmesh_vind(MESH<FLOAT_TYPE>& mesh, cv::Mat& vind, int width, int height, MESH<FLOAT_TYPE>& submesh) {

	//MESH<FLOAT_TYPE> submesh;
	int vnum = mesh.number;

	std::vector<bool> validv(vnum, false);


	for (int i = 0; i < vind.rows; i++)
	{
		// cout << vind.at<Point>(i).x << endl;
		// cout << vind.at<Point>(i).y << endl;
		int temp_ind = vind.at<cv::Point>(i).x*height + vind.at<cv::Point>(i).y;
		validv[temp_ind] = true;
		// cout << validv[temp_ind] << endl;
	}

	int subvnum = vind.rows;
	std::vector<int> allind(vnum, 0);

	for (int i = 0; i < subvnum; i++)
	{

		int temp_ind = vind.at<cv::Point>(i).x*height + vind.at<cv::Point>(i).y;
		allind[temp_ind] = i;

	}

	/////////////////////// step1 :determine if a triangle is valid

	std::vector<cv::Vec3i> subi;

	for (int i = 0; i < mesh.t_number; i++)
	{
		if (validv[mesh.T[i * 3 + 0]] == true && validv[mesh.T[i * 3 + 1]] == true && validv[mesh.T[i * 3 + 2]] == true) {

			cv::Vec3i t;
			t[0] = mesh.T[i * 3 + 0];
			t[1] = mesh.T[i * 3 + 1];
			t[2] = mesh.T[i * 3 + 2];
			subi.push_back(t);

		}
	}
	// change the face index to new indices in vind


	for (int i = 0; i < subi.size(); i++)
	{
		subi[i][0] = allind[subi[i][0]];
		subi[i][1] = allind[subi[i][1]];
		subi[i][2] = allind[subi[i][2]];
	}



	/////////////////////// step2: determine if a vertex is referenced in the valid triangles

	//is vertex from step1 an unreference point?, valid is the same size of vind

	std::vector<bool> valid(subvnum, false);

	for (int i = 0; i < subi.size(); i++)
	{
		valid[subi[i][0]] = true;
		valid[subi[i][1]] = true;
		valid[subi[i][2]] = true;
	}

	///////////////////// compute the subv and its ind, ind2
	int start_id = 0;
	std::vector<int> ind;

	std::vector<int> allind2(subvnum, 0);
	for (int i = 0; i < valid.size(); i++)
	{
		if (valid[i] == true)
		{
			int temp_ind = vind.at<cv::Point>(i).x*height + vind.at<cv::Point>(i).y;
			ind.push_back(temp_ind);
			submesh.X[start_id * 3 + 0] = mesh.X[temp_ind * 3 + 0];
			submesh.X[start_id * 3 + 1] = mesh.X[temp_ind * 3 + 1];
			submesh.X[start_id * 3 + 2] = mesh.X[temp_ind * 3 + 2];

			submesh.VT[start_id * 2 + 0] = mesh.VT[temp_ind * 2 + 0];
			submesh.VT[start_id * 2 + 1] = mesh.VT[temp_ind * 2 + 1];

			allind2[i] = start_id;
			start_id++;

		}



	}
	submesh.number = start_id;
	///////////////////// recompute the face index to new indices in subv
	int subvnum_final = start_id;
	submesh.vt_number = start_id;

	submesh.t_number = subi.size();
	for (int i = 0; i < subi.size(); i++)
	{


		// Vec3i t;
		/// subi[i][0] = allind2[subi[i][0]];
		// subi[i][1] = allind2[subi[i][1]];
		// subi[i][2] = allind2[subi[i][2]];
		submesh.T[i * 3 + 0] = allind2[subi[i][0]];
		submesh.T[i * 3 + 1] = allind2[subi[i][1]];
		submesh.T[i * 3 + 2] = allind2[subi[i][2]];


	}

	///// recompute the texture coordinates, normals if needed
	/* int vt_num = mesh.vt_number;
	submesh.vt_number = mesh.vt_number;
	if (vt_num > 0)
	{
	for (int i = 0; i< vt_num; i++)
	{
	submesh.VT[i * 2 + 0] = mesh.VT[i * 2 + 0];
	submesh.VT[i * 2 + 1] = mesh.VT[i * 2 + 2];
	// cout << submesh.VT[i * 2 + 0] << ", " << submesh.VT[i * 2 + 1] << endl;
	}

	}*/

	// submesh.Write_OBJ("test2.obj");

	//return submesh;

}

void pointmap2mesh(cv::Mat& pointmap, cv::Mat& mask, MESH<FLOAT_TYPE>& mesh) {

	//MESH<FLOAT_TYPE>		mesh;
	int m = pointmap.rows;//height
	int n = pointmap.cols;//width

	cv::Mat channel[3];
	split(pointmap, channel);

	cv::Mat channel_t[3];

	channel_t[0] = channel[0].t();
	channel_t[1] = channel[1].t();
	channel_t[2] = channel[2].t();

	cv::Mat pointmap_permute;

	cv::merge(channel_t, 3, pointmap_permute);


	cv::Mat v = pointmap_permute.reshape(1, m*n);


	cv::Mat umap, vmap;

	meshgridTestG(cv::Range(0.5, n - 0.5), cv::Range(0.5, m - 0.5), umap, vmap);

	cv::Mat channel_uv[2];

	channel_uv[0] = umap.t();
	channel_uv[1] = vmap.t();
	cv::Mat temp;
	cv::merge(channel_uv, 2, temp);


	// cout << temp.type() << endl;
	cv::Mat uv = temp.reshape(1, m*n);
	// cout << uv.type() << endl;
	cv::Mat vt = uv.clone();

	vt.convertTo(vt, CV_32FC2);



	for (int i = 0; i < vt.rows; i++)
	{


		vt.at<float>(i, 0) = vt.at<float>(i, 0) / n;
		vt.at<float>(i, 1) = 1 - vt.at<float>(i, 1) / m;


	}

	// generate faces

	cv::Mat f = cv::Mat::zeros((m - 1)*(n - 1) * 2, 3, CV_32F);


	for (int i = 1; i <= n - 1; i++)
	{
		for (int j = 1; j <= m - 1; j++)
		{
			int t1 = (i - 1)*m + j - 1;
			int t2 = t1 + m;


			f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2, 0) = t2;
			f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2 + 1, 0) = t2 + 1;

			f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2, 1) = t1;
			f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2 + 1, 1) = t1;

			f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2, 2) = t2 + 1;
			f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2 + 1, 2) = t1 + 1;

		}
	}

	// fill mesh

	mesh.number = v.rows;
	mesh.vt_number = v.rows;
	mesh.t_number = f.rows;
	for (int i = 0; i < mesh.number; i++)
	{
		mesh.X[i * 3 + 0] = v.at<float>(i, 0);
		mesh.X[i * 3 + 1] = v.at<float>(i, 1);
		mesh.X[i * 3 + 2] = v.at<float>(i, 2);



		mesh.VT[i * 2 + 0] = vt.at<float>(i, 0);
		mesh.VT[i * 2 + 1] = vt.at<float>(i, 1);

		// if (mesh.VT[i * 2 + 0] != 0){
		// cout << vt.type() << endl;
		//	cout << mesh.VT[i * 2 + 0] << ", " << mesh.VT[i * 2 + 1] << endl;
		// }

	}


	for (int i = 0; i < mesh.t_number; i++)
	{
		mesh.T[i * 3 + 0] = f.at<float>(i, 0);
		mesh.T[i * 3 + 1] = f.at<float>(i, 1);
		mesh.T[i * 3 + 2] = f.at<float>(i, 2);




	}



	// extract the non zeros pixels in mask
	/*
	cv::Mat locations;
	findNonZero(mask, locations);
	// MESH<FLOAT_TYPE> submesh;
	if (locations.rows != v.rows) {


		selectmesh_vind(mesh, locations, n, m, submesh);


	}
	*/


}

void pointmap2mesh_counting(cv::Mat& pointmap, MESH<FLOAT_TYPE>& mesh, int & times, cv::Mat & vt, cv::Mat & f, Camera& cam) {
//void pointmap2mesh_counting(cv::Mat& pointmap, cv::Mat& mask, MESH<FLOAT_TYPE>& mesh, int & times, cv::Mat & vt, cv::Mat & f) {

	//MESH<FLOAT_TYPE>		mesh;
	int m = pointmap.rows;//height
	int n = pointmap.cols;//width

	cv::Mat channel[3];
	split(pointmap, channel);

	cv::Mat channel_t[3];

	channel_t[0] = channel[0].t();
	channel_t[1] = channel[1].t();
	channel_t[2] = channel[2].t();

	cv::Mat pointmap_permute;

	cv::merge(channel_t, 3, pointmap_permute);


	cv::Mat v = pointmap_permute.reshape(1, m*n);
	//std::cout << "times: " << times << "\n";
	if (times < 6)
	{
		cv::Mat umap, vmap;

		meshgridTestG(cv::Range(0.5, n - 0.5), cv::Range(0.5, m - 0.5), umap, vmap);

		cv::Mat channel_uv[2];

		channel_uv[0] = umap.t();
		channel_uv[1] = vmap.t();
		cv::Mat temp;
		cv::merge(channel_uv, 2, temp);


		// cout << temp.type() << endl;
		cv::Mat uv = temp.reshape(1, m*n);
		// cout << uv.type() << endl;
		//cv::Mat vt = uv.clone();
		vt = uv.clone();

		vt.convertTo(vt, CV_32FC2);




		for (int i = 0; i < vt.rows; i++)
		{


			vt.at<float>(i, 0) = vt.at<float>(i, 0) / n;
			vt.at<float>(i, 1) = 1 - vt.at<float>(i, 1) / m;

			mesh.VT[i * 2 + 0] = vt.at<float>(i, 0);
			mesh.VT[i * 2 + 1] = vt.at<float>(i, 1);
		}

		// generate faces

		//cv::Mat f = cv::Mat::zeros((m - 1)*(n - 1) * 2, 3, CV_32F);
		f = cv::Mat::zeros((m - 1)*(n - 1) * 2, 3, CV_32F);


		for (int i = 1; i <= n - 1; i++)
		{
			for (int j = 1; j <= m - 1; j++)
			{
				int t1 = (i - 1)*m + j - 1;
				int t2 = t1 + m;


				f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2, 0) = t2;
				f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2 + 1, 0) = t2 + 1;

				f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2, 1) = t1;
				f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2 + 1, 1) = t1;

				f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2, 2) = t2 + 1;
				f.at<float>((i - 1)*(m - 1) * 2 + (j - 1) * 2 + 1, 2) = t1 + 1;

			}
		}
		times++;
	}


	// fill mesh

	mesh.number = v.rows;
	mesh.vt_number = v.rows;
	mesh.t_number = f.rows;

	Eigen::Vector3f a;
	Eigen::Vector3f pointWorld;

//#pragma omp parallel for
	for (int i = 0; i < mesh.number; i++)
	{
		/*
		a[0] = v.at<float>(i, 0);
		a[1] = v.at<float>(i, 1);
		a[2] = v.at<float>(i, 2);

		pointWorld = cam.Rt_curr_to_world.block<3, 3>(0, 0) * a + cam.Rt_curr_to_world.block<3, 1>(0, 3);

		mesh.X[i * 3 + 0] = pointWorld[0];
		mesh.X[i * 3 + 1] = -pointWorld[1];
		mesh.X[i * 3 + 2] = -pointWorld[2];
		*/
		mesh.X[i * 3 + 0] = v.at<float>(i, 0);
		mesh.X[i * 3 + 1] = v.at<float>(i, 1);
		mesh.X[i * 3 + 2] = v.at<float>(i, 2);
		// if (mesh.VT[i * 2 + 0] != 0){
		// cout << vt.type() << endl;
		//	cout << mesh.VT[i * 2 + 0] << ", " << mesh.VT[i * 2 + 1] << endl;
		// }

	}

	/*
//#pragma omp parallel for
	for (int i = 0; i < mesh.t_number; i++)
	{
		mesh.T[i * 3 + 0] = f.at<float>(i, 0);
		mesh.T[i * 3 + 1] = f.at<float>(i, 1);
		mesh.T[i * 3 + 2] = f.at<float>(i, 2);




	}
	*/

	// Remove long face
	int face_counter = 0;
	float edge_length[3];
	int ind_v[3];
	float threshold = 0.01;

	for (int i = 0; i < mesh.t_number; i++)
	{
		ind_v[0] = f.at<float>(i, 0);
		ind_v[1] = f.at<float>(i, 1);
		ind_v[2] = f.at<float>(i, 2);

		cv::Point3f v0(mesh.X[ind_v[0] * 3 + 0], mesh.X[ind_v[0] * 3 + 1], mesh.X[ind_v[0] * 3 + 2]);
		cv::Point3f v1(mesh.X[ind_v[1] * 3 + 0], mesh.X[ind_v[1] * 3 + 1], mesh.X[ind_v[1] * 3 + 2]);
		cv::Point3f v2(mesh.X[ind_v[2] * 3 + 0], mesh.X[ind_v[2] * 3 + 1], mesh.X[ind_v[2] * 3 + 2]);

		cv::Point3f edge1(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
		cv::Point3f edge2(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
		cv::Point3f edge3(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);

		//edge_length[0] = sqrt(edge1.x*edge1.x + edge1.y*edge1.y + edge1.z*edge1.z);
		//edge_length[1] = sqrt(edge2.x*edge2.x + edge2.y*edge2.y + edge2.z*edge2.z);
		//edge_length[2] = sqrt(edge3.x*edge3.x + edge3.y*edge3.y + edge3.z*edge3.z);

		edge_length[0] = (edge1.x*edge1.x + edge1.y*edge1.y + edge1.z*edge1.z);
		edge_length[1] = (edge2.x*edge2.x + edge2.y*edge2.y + edge2.z*edge2.z);
		edge_length[2] = (edge3.x*edge3.x + edge3.y*edge3.y + edge3.z*edge3.z);

		auto smallest = std::min_element(std::begin(edge_length), std::end(edge_length));

		auto biggest = std::max_element(std::begin(edge_length), std::end(edge_length));

		if (*smallest > 0 && *biggest < threshold)
		{
			mesh.T[face_counter * 3 + 0] = f.at<float>(i, 0);
			mesh.T[face_counter * 3 + 1] = f.at<float>(i, 1);
			mesh.T[face_counter * 3 + 2] = f.at<float>(i, 2);
			face_counter++;
		}
	}

	mesh.t_number = face_counter;
	
	std::stringstream cam1_file, cam2_file;

	cam1_file << "pc2mesh0.obj";
	cam2_file << "pc2mesh1.obj";

	if (cam.id == 0)
		mesh.Write_OBJ(cam1_file.str().c_str());
	if (cam.id == 1)
		mesh.Write_OBJ(cam2_file.str().c_str());
	//Sleep(3000);
	
	// extract the non zeros pixels in mask
	/*
	cv::Mat locations;
	findNonZero(mask, locations);
	// MESH<FLOAT_TYPE> submesh;
	if (locations.rows != v.rows) {


		selectmesh_vind(mesh, locations, n, m, submesh);


	}
	*/


}


void RemoveLongFace(MESH<FLOAT_TYPE>& mesh, float threshold, MESH<FLOAT_TYPE>& mesh_clean) {

	int face_counter = 0;
	float edge_length[3];
	int ind_v[3];

//#pragma omp parallel for shared(mesh, face_counter, mesh_clean)
	for (int i = 0; i < mesh.t_number; i++)
	{
		ind_v[0] = mesh.T[i * 3 + 0];
		ind_v[1] = mesh.T[i * 3 + 1];
		ind_v[2] = mesh.T[i * 3 + 2];

		cv::Point3f v0(mesh.X[ind_v[0] * 3 + 0], mesh.X[ind_v[0] * 3 + 1], mesh.X[ind_v[0] * 3 + 2]);
		cv::Point3f v1(mesh.X[ind_v[1] * 3 + 0], mesh.X[ind_v[1] * 3 + 1], mesh.X[ind_v[1] * 3 + 2]);
		cv::Point3f v2(mesh.X[ind_v[2] * 3 + 0], mesh.X[ind_v[2] * 3 + 1], mesh.X[ind_v[2] * 3 + 2]);

		cv::Point3f edge1(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
		cv::Point3f edge2(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
		cv::Point3f edge3(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);

		//edge_length[0] = sqrt(edge1.x*edge1.x + edge1.y*edge1.y + edge1.z*edge1.z);
		//edge_length[1] = sqrt(edge2.x*edge2.x + edge2.y*edge2.y + edge2.z*edge2.z);
		//edge_length[2] = sqrt(edge3.x*edge3.x + edge3.y*edge3.y + edge3.z*edge3.z);
		
		edge_length[0] = (edge1.x*edge1.x + edge1.y*edge1.y + edge1.z*edge1.z);
		edge_length[1] = (edge2.x*edge2.x + edge2.y*edge2.y + edge2.z*edge2.z);
		edge_length[2] = (edge3.x*edge3.x + edge3.y*edge3.y + edge3.z*edge3.z);

		auto smallest = std::min_element(std::begin(edge_length), std::end(edge_length));

		auto biggest = std::max_element(std::begin(edge_length), std::end(edge_length));

		if (*smallest > 0 && *biggest < threshold)
		{
			mesh_clean.T[face_counter * 3 + 0] = mesh.T[i * 3 + 0];
			mesh_clean.T[face_counter * 3 + 1] = mesh.T[i * 3 + 1];
			mesh_clean.T[face_counter * 3 + 2] = mesh.T[i * 3 + 2];
			face_counter++;
		}



	}

	mesh_clean.t_number = face_counter;
	mesh_clean.number = mesh.number;
	mesh_clean.vt_number = mesh.vt_number;

	
//#pragma omp parallel for shared(mesh, mesh_clean)
	for (int i = 0; i < mesh.number; i++)
	{
		mesh_clean.X[i * 3 + 0] = mesh.X[i * 3 + 0];
		mesh_clean.X[i * 3 + 1] = mesh.X[i * 3 + 1];
		mesh_clean.X[i * 3 + 2] = mesh.X[i * 3 + 2];



		mesh_clean.VT[i * 2 + 0] = mesh.VT[i * 2 + 0];
		mesh_clean.VT[i * 2 + 1] = mesh.VT[i * 2 + 1];

		// cout << mesh_clean.VT[i * 2 + 0] << ", " << mesh_clean.VT[i * 2 + 1] << endl;
	}
	

	// mesh.Write_OBJ("test6.obj");
	// mesh_clean.Write_OBJ("test7.obj");

}
