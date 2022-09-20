#include "triangulation.h"
#include "fileprocessor.h"

CameraGroup::CameraGroup(std::string filepath)
{
    std::vector<std::string> lines = ReadLine(txt2String(filepath));
    CameraCount = std::stoi(Split(lines[0], "=")[1]);
    for (int i = 0; i < CameraCount; i++)
    {
        Camera cam;
        int index = 1 + (i * 4);
        Read_2DArray(cam.K, lines[index + 1]);
        cam.K_Inverse = cam.K.inverse();
        Read_2DArray(cam.Rt, lines[index + 2]);
		cam.R = cam.Rt.block(0, 0, 3, 3);
		cam.t = cam.Rt.block(0, 3, 3, 1);
        Read_2DArray(cam.D, lines[index + 3]);
		CameraInfo.push_back(cam);
    }
}

void CameraGroup::Triangulation(const double distance_tol)
{
	std::vector<std::vector<Eigen::Vector3d>> F_Group;
	std::vector<std::vector<Eigen::Vector2d>>::iterator CameraPoints_ptr;
	int cam = 0;

	for(CameraPoints_ptr = CameraPoints.begin(); CameraPoints_ptr < CameraPoints.end(); CameraPoints_ptr++, cam++)
	{
		std::vector<Eigen::Vector3d> f_group;
		std::vector<Eigen::Vector2d>::iterator Points_ptr;
		for(Points_ptr = CameraPoints_ptr->begin(); Points_ptr < CameraPoints_ptr->end(); Points_ptr++)
		{
			Eigen::MatrixXd w(3, 1), f(3, 1);
			Eigen::Vector2d Point = *Points_ptr;
			w << Point[0], Point[1], 1;
			f << CameraInfo[cam].K_Inverse * w;
			f_group.push_back(f);
		}
		F_Group.push_back(f_group);
	}

	std::vector<std::vector<Eigen::Vector3d>>::iterator F_maincam_ptr;
	int maincam = 0;
	for (F_maincam_ptr = F_Group.begin(); F_maincam_ptr < F_Group.end(); F_maincam_ptr++, maincam++)
	{
		int subcam = 1;
		for (std::vector<std::vector<Eigen::Vector3d>>::iterator F_subcam_ptr = F_maincam_ptr + 1; F_subcam_ptr < F_Group.end(); F_subcam_ptr++, subcam++)
		{
			for (std::vector<Eigen::Vector3d>::iterator F_maincampoint_ptr = F_maincam_ptr->begin(); F_maincampoint_ptr < F_maincam_ptr->end(); F_maincampoint_ptr++)
			{
				Eigen::Vector3d F_m_p = *F_maincampoint_ptr;
				for (std::vector<Eigen::Vector3d>::iterator F_subcampoint_ptr = F_subcam_ptr->begin(); F_subcampoint_ptr < F_subcam_ptr->end(); F_subcampoint_ptr++)
				{
					Eigen::Vector3d F_s_p = *F_subcampoint_ptr;
					Eigen::MatrixXd S_star(2, 2), H(3, 2), S_star_temp(2, 1), W_star(2, 6), F_star(2, 6), W_sub(1, 3), W_main(1, 3), W_world_sub(3, 1), W_world_main(3, 1), W_world_dist(3, 1);
					H << F_m_p, -(CameraInfo[subcam].R * F_s_p);
					S_star_temp << (H.transpose() * H).inverse() * (H.transpose() * CameraInfo[subcam].t);
					S_star << S_star_temp(0, 0), 0, 0, S_star_temp(1, 0);
					F_star << F_m_p(0), F_m_p(1), F_m_p(2), 0, 0, 0,
							  0, 0, 0, F_s_p(0), F_s_p(1), F_s_p(2);
					W_star << S_star * F_star;
					W_world_sub << (CameraInfo[subcam].R * W_star.block(0, 0, 1, 3).transpose()) + CameraInfo[subcam].t;
					W_world_main << (CameraInfo[maincam].R * W_star.block(1, 3, 1, 3).transpose()) + CameraInfo[maincam].t;
					W_world_dist << W_world_sub - W_world_main;
					double dist = W_world_dist.norm();
					if (dist < distance_tol)
					{
						Eigen::Vector3d tri_point;
						tri_point << (W_world_sub + W_world_main) / 2;
						TriangulatedPoints.push_back(tri_point);
					}
				}
			}
		}
	}
}

void CameraGroup::AddCameraPoints(const std::vector<Eigen::Vector2d>& camerapoints)
{
	CameraPoints.push_back(camerapoints);
}

void CameraGroup::ClearCameraPoints()
{
	CameraPoints.clear();
}

void CameraGroup::ClearTriangulatedPoints()
{
	TriangulatedPoints.clear();
	TriangulatedScore.clear();
}

void CameraGroup::TriangulationCondense(const double distance_tol)
{
	std::vector<Eigen::Vector3d> tri_points_condensed;
	std::vector<double> tri_points_score;

	for (std::vector<Eigen::Vector3d>::iterator TriangulatedPoints_main_ptr = TriangulatedPoints.begin(); TriangulatedPoints_main_ptr < TriangulatedPoints.end(); TriangulatedPoints_main_ptr++)
	{
		int condense_points_count = 0;
		double dist_sum = 1;
		Eigen::Vector3d tri_point_sum = Eigen::Vector3d::Zero();
		for (std::vector<Eigen::Vector3d>::iterator TriangulatedPointss_sub_ptr = TriangulatedPoints.begin(); TriangulatedPointss_sub_ptr < TriangulatedPoints.end(); TriangulatedPointss_sub_ptr++)
		{
			double dist = (*TriangulatedPoints_main_ptr - *TriangulatedPointss_sub_ptr).norm();
			
			if (dist < distance_tol)
			{
				tri_point_sum += *TriangulatedPointss_sub_ptr;
				condense_points_count++;
				dist_sum += dist * dist;
			}
		}
		if (condense_points_count > 1)
		{
			bool isrepeated = false;
			for (std::vector<Eigen::Vector3d>::iterator tri_points_condensed_ptr = tri_points_condensed.begin(); tri_points_condensed_ptr < tri_points_condensed.end(); tri_points_condensed_ptr++)
			{
				if ((tri_point_sum / condense_points_count - *tri_points_condensed_ptr).norm() < 0.0001)
				{
					isrepeated = true;
					break;
				}
			}
			if (!isrepeated)
			{
				tri_points_condensed.push_back(tri_point_sum / condense_points_count);
				tri_points_score.push_back(condense_points_count / dist_sum);
			}
		}
	}
	if (tri_points_condensed.size() > 0)
	{
		TriangulatedPoints = tri_points_condensed;
		TriangulatedScore = tri_points_score;
	}
}

void CameraGroup::TriangulationWorkspaceFilter(const std::vector<double> workspace)
{
	std::vector<Eigen::Vector3d> tri_points_filtered;
	for (std::vector<Eigen::Vector3d>::iterator TriangulatedPoints_ptr = TriangulatedPoints.begin(); TriangulatedPoints_ptr < TriangulatedPoints.end(); TriangulatedPoints_ptr++)
	{
		Eigen::Vector3d tri_point_unfiltered = *TriangulatedPoints_ptr;
		if (tri_point_unfiltered[0] > workspace[0] && tri_point_unfiltered[0] < workspace[1] &&
			tri_point_unfiltered[1] > workspace[2] && tri_point_unfiltered[1] < workspace[3] &&
			tri_point_unfiltered[2] > workspace[4] && tri_point_unfiltered[2] < workspace[5])
			tri_points_filtered.push_back(tri_point_unfiltered);
	}
	TriangulatedPoints = tri_points_filtered;
}