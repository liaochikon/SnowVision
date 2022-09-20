#include "SnowVision.h"
#include "BasicClasses.h"
#include "Triangulation.h"

void ImagePoints2TriangulatedPoints(std::vector<Eigen::Vector3d>& TriangulatedPoints, const std::vector<std::vector<Eigen::Vector2d>>& ImagePoints, const CameraGroup Cameras, const double distance_tol)
{
	std::vector<std::vector<Eigen::Vector3d>> F_Group;
	Get_F_Group(F_Group, ImagePoints, Cameras);
	std::vector<std::vector<Eigen::Vector3d>>::iterator F_maincam_ptr;
	int maincam = 0;
	for (F_maincam_ptr = F_Group.begin(); F_maincam_ptr < F_Group.end(); F_maincam_ptr++, maincam++)
	{
		std::vector<Eigen::Vector3d> F_m = *F_maincam_ptr;
		int subcam = 1;
		for (std::vector<std::vector<Eigen::Vector3d>>::iterator F_subcam_ptr = F_maincam_ptr+1; F_subcam_ptr < F_Group.end(); F_subcam_ptr++, subcam++)
		{
			std::vector<Eigen::Vector3d> F_s = *F_subcam_ptr;
			for (std::vector<Eigen::Vector3d>::iterator F_maincampoint_ptr = F_m.begin(); F_maincampoint_ptr < F_m.end(); F_maincampoint_ptr++)
			{
				Eigen::Vector3d F_m_p = *F_maincampoint_ptr;
				for (std::vector<Eigen::Vector3d>::iterator F_subcampoint_ptr = F_s.begin(); F_subcampoint_ptr < F_s.end(); F_subcampoint_ptr++)
				{
					Eigen::Vector3d F_s_p = *F_subcampoint_ptr;
					Eigen::MatrixXd S_star(2, 2);
					//Get_S_Star(S_star, F_m_p, F_s_p, Cameras.CameraInfo[subcam].Rt);
					Eigen::MatrixXd R_sub, t_sub;
					Split_Camera_Rt(R_sub, t_sub, Cameras.CameraInfo[subcam].Rt);
					Eigen::MatrixXd H(3, 2);
					H << F_m_p, -(R_sub * F_s_p);
					Eigen::MatrixXd S_star_temp(2, 1);
					S_star_temp << (H.transpose() * H).inverse() * (H.transpose() * t_sub);
					S_star << S_star_temp(0, 0), 0, 0, S_star_temp(1, 0);

					Eigen::MatrixXd W_star(2, 6);
					Eigen::MatrixXd F_star(2, 6);
					F_star << F_m_p(0), F_m_p(1), F_m_p(2), 0, 0, 0,
								0, 0, 0, F_s_p(0), F_s_p(1), F_s_p(2);
					W_star << S_star * F_star;

					Eigen::MatrixXd W_sub(1, 3), W_main(1, 3);
					W_sub << W_star.block(0, 0, 1, 3);
					W_main << W_star.block(1, 3, 1, 3);

					Eigen::MatrixXd W_world_sub(3, 1), W_world_main(3, 1);
					W_TO_World_Coordination(W_world_sub, W_sub, Cameras.CameraInfo[subcam].Rt);
					W_TO_World_Coordination(W_world_main, W_main, Cameras.CameraInfo[maincam].Rt);

					Eigen::MatrixXd W_world_dist(3, 1);
					W_world_dist << W_world_sub - W_world_main;
					double dist = W_world_dist.norm();

					if (dist < distance_tol)
					{
						Eigen::Vector3d tri_point;
						tri_point << (W_world_sub + W_world_main) / 2;
						//std::cout << "tri_point" << "\n";
						//std::cout << tri_point << "\n";
						TriangulatedPoints.push_back(tri_point);
					}
				}
			}
		}
	}
}

void Split_Camera_Rt(Eigen::MatrixXd& output_R, Eigen::MatrixXd& output_t, Eigen::MatrixXd Rt)
{
	output_R = Rt.block(0, 0, 3, 3);
	output_t = Rt.block(0, 3, 3, 1);
}

void Get_H(Eigen::MatrixXd& output, Eigen::Vector3d f_main, Eigen::Vector3d f_sub, Eigen::MatrixXd R_sub)
{
	Eigen::MatrixXd g(3, 1);
	g << R_sub * f_sub;
	output << f_main, -g;
}

void Get_S_Star(Eigen::MatrixXd& output, const Eigen::Vector3d& f_main, const Eigen::Vector3d& f_sub, const Eigen::MatrixXd& Rt)
{
	Eigen::MatrixXd R_sub, t_sub;
	Split_Camera_Rt(R_sub, t_sub, Rt);
	Eigen::MatrixXd H(3, 2);
	H << f_main, -(R_sub * f_sub);
	Eigen::MatrixXd S_star_temp(2, 1);
	S_star_temp << (H.transpose() * H).inverse() * (H.transpose() * t_sub);
	output << S_star_temp(0, 0), 0                ,
			  0                , S_star_temp(1, 0);
}

void Get_W_star(Eigen::MatrixXd& output, Eigen::MatrixXd s_star, Eigen::Vector3d f_main, Eigen::MatrixXd f_sub)
{
	Eigen::MatrixXd F_star(2, 6);
	F_star << f_main(0), f_main(1), f_main(2),        0,        0,        0,
			          0,         0,         0, f_sub(0), f_sub(1), f_sub(2);
	output << s_star * F_star;
}

void Get_W_sub_W_main(Eigen::MatrixXd& output1, Eigen::MatrixXd& output2, Eigen::MatrixXd w_star)
{
	output1 << w_star.block(0, 0, 1, 3);
	output2 << w_star.block(1, 3, 1, 3);
}

void W_TO_World_Coordination(Eigen::MatrixXd& output, Eigen::MatrixXd w, Eigen::MatrixXd Rt)
{
	Eigen::MatrixXd R, t;
	Split_Camera_Rt(R, t, Rt);
	Eigen::MatrixXd W(3, 1);
	W << R * w.transpose();
	output << W + t;
}

void Get_F_Group(std::vector<std::vector<Eigen::Vector3d>>& output, std::vector<std::vector<Eigen::Vector2d>> Points, CameraGroup Cameras)
{
	for (int i = 0; i < Points.size(); i++)
	{
		std::vector<Eigen::Vector3d> f_group;
		for (int j = 0; j < Points[i].size(); j++)
		{
			Eigen::MatrixXd w(3,1);
			w << Points[i][j][0], Points[i][j][1], 1;
			Eigen::MatrixXd f(3, 1);
			f << Cameras.CameraInfo[i].K_Inverse * w;
			f_group.push_back(f);
		}
		output.push_back(f_group);
	}
}