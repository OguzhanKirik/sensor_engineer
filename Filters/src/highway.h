/* \author Aaron Brown */
// Handle logic for creating traffic on highway and animating it

#include "render/render.h"
#include "sensors/lidar.h"
#include "tools.h"
#include "filters/iekf/iekf.hpp"
#include "filters/rts/ekf_rts_smoother.h"
#include "filters/lag_smoother/ukf_fixed_lag_smoother.h"
#include "filters/rts/ukf_rts_smoother.h"

class Highway
{
public:

	std::vector<Car> traffic;
	Car egoCar;
	Tools tools;
	bool pass = true;
	std::vector<double> rmseThreshold = {0.30,0.16,0.95,0.70};
	std::vector<double> rmseFailLog = {0.0,0.0,0.0,0.0};
	Lidar* lidar;
	
	// Parameters 
	// --------------------------------
	// Set which cars to track with UKF
	std::vector<bool> trackCars = {true,true,true};
	// Use EKF instead of UKF (false = UKF, true = EKF)
	bool use_ekf = true;
	// Run offline RTS smoothing on top of EKF after the simulation ends
	bool use_ekf_rts = false;
	// Run offline RTS smoothing on top of UKF after the simulation ends
	bool use_ukf_rts = false;
	// Run fixed-lag smoothing on top of UKF after the simulation ends
	bool use_ukf_fixed_lag = false;
	// Use IEKF (Iterated Extended Kalman Filter)
	bool use_iekf = false;
	// Use CKF (Cubature Kalman Filter)
	bool use_ckf = false;
	// Use PF (Particle Filter)
	bool use_pf = false;
	// Use MHE (Moving Horizon Estimation scaffold)
	bool use_mhe = false;
	// Visualize sensor measurements
	bool visualize_lidar = true;
	bool visualize_radar = true;
	bool visualize_pcd = false;
	// Predict path in the future using UKF
	double projectedTime = 0;
	int projectedSteps = 0;
	int fixed_lag_steps = 30;
	bool rts_reported = false;
	EKFRTSSmoother ekf_rts_smoother;
	UKFFixedLagSmoother ukf_fixed_lag_smoother;
	UKFRTSSmoother ukf_rts_smoother;
	std::vector<std::vector<VectorXd>> per_car_ground_truth_;
	// --------------------------------

	Highway(pcl::visualization::PCLVisualizer::Ptr& viewer)
	{
		#ifdef USE_EKF
		use_ekf = true;
		use_ekf_rts = false;
		use_ukf_rts = false;
		use_ukf_fixed_lag = false;
		use_iekf = false;
		use_ckf = false;
		use_pf = false;
		use_mhe = false;
		#elif defined(USE_EKF_RTS)
		use_ekf = true;
		use_ekf_rts = true;
		use_ukf_rts = false;
		use_ukf_fixed_lag = false;
		use_iekf = false;
		use_ckf = false;
		use_pf = false;
		use_mhe = false;
		#elif defined(USE_UKF_RTS)
		use_ekf = false;
		use_ekf_rts = false;
		use_ukf_rts = true;
		use_ukf_fixed_lag = false;
		use_iekf = false;
		use_ckf = false;
		use_pf = false;
		use_mhe = false;
		#elif defined(USE_UKF_FIXED_LAG)
		use_ekf = false;
		use_ekf_rts = false;
		use_ukf_rts = false;
		use_ukf_fixed_lag = true;
		use_iekf = false;
		use_ckf = false;
		use_pf = false;
		use_mhe = false;
		#elif defined(USE_IEKF)
		use_ekf = false;
		use_ekf_rts = false;
		use_ukf_rts = false;
		use_ukf_fixed_lag = false;
		use_iekf = true;
		use_ckf = false;
		use_pf = false;
		use_mhe = false;
		#elif defined(USE_MHE)
		use_ekf = false;
		use_ekf_rts = false;
		use_ukf_rts = false;
		use_ukf_fixed_lag = false;
		use_iekf = false;
		use_ckf = false;
		use_pf = false;
		use_mhe = true;
		#elif defined(USE_UKF)
		use_ekf = false;
		use_ekf_rts = false;
		use_ukf_rts = false;
		use_ukf_fixed_lag = false;
		use_iekf = false;
		use_ckf = false;
		use_pf = false;
		use_mhe = false;
		#elif defined(USE_CKF)
		use_ckf = true;
		use_ekf = false;
		use_ekf_rts = false;
		use_ukf_rts = false;
		use_ukf_fixed_lag = false;
		use_iekf = false;
		use_pf = false;
		use_mhe = false;
		#elif defined(USE_PF)
		use_pf = true;
		use_ekf = false;
		use_ekf_rts = false;
		use_ukf_rts = false;
		use_ukf_fixed_lag = false;
		use_iekf = false;
		use_ckf = false;
		use_mhe = false;
		#endif

		tools = Tools();
		per_car_ground_truth_.resize(trackCars.size());
	
		egoCar = Car(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), 0, 0, 2, "egoCar");
		
		Car car1(Vect3(-10, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), 5, 0, 2, "car1");
		
		std::vector<accuation> car1_instructions;
		accuation a = accuation(0.5*1e6, 0.5, 0.0);
		car1_instructions.push_back(a);
		a = accuation(2.2*1e6, 0.0, -0.2);
		car1_instructions.push_back(a);
		a = accuation(3.3*1e6, 0.0, 0.2);
		car1_instructions.push_back(a);
		a = accuation(4.4*1e6, -2.0, 0.0);
		car1_instructions.push_back(a);
	
		car1.setInstructions(car1_instructions);
		if( trackCars[0] )
		{
			if(use_pf)
			{
				PF pf1;
				car1.setPF(pf1);
			}
			else if(use_mhe)
			{
				MHE mhe1;
				car1.setMHE(mhe1);
			}
			else if(use_ckf)
			{
				CKF ckf1;
				car1.setCKF(ckf1);
			}
			else if(use_iekf)
			{
				IEKF iekf1;
				car1.setIEKF(iekf1);
			}
			else if(use_ekf)
			{
				EKF ekf1;
				car1.setEKF(ekf1);
			}
			else
			{
				UKF ukf1;
				car1.setUKF(ukf1);
			}
		}
		traffic.push_back(car1);
		
		Car car2(Vect3(25, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), -6, 0, 2, "car2");
		std::vector<accuation> car2_instructions;
		a = accuation(4.0*1e6, 3.0, 0.0);
		car2_instructions.push_back(a);
		a = accuation(8.0*1e6, 0.0, 0.0);
		car2_instructions.push_back(a);
		car2.setInstructions(car2_instructions);
		if( trackCars[1] )
		{
			if(use_pf)
			{
				PF pf2;
				car2.setPF(pf2);
			}
			else if(use_mhe)
			{
				MHE mhe2;
				car2.setMHE(mhe2);
			}
			else if(use_ckf)
			{
				CKF ckf2;
				car2.setCKF(ckf2);
			}
			else if (use_iekf)
			{
				IEKF iekf2;
				car2.setIEKF(iekf2);
			}
			else if(use_ekf)
			{
				EKF ekf2;
				car2.setEKF(ekf2);
			}
			else
			{
				UKF ukf2;
				car2.setUKF(ukf2);
			}
		}
		traffic.push_back(car2);
	
		Car car3(Vect3(-12, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), 1, 0, 2, "car3");
		std::vector<accuation> car3_instructions;
		a = accuation(0.5*1e6, 2.0, 1.0);
		car3_instructions.push_back(a);
		a = accuation(1.0*1e6, 2.5, 0.0);
		car3_instructions.push_back(a);
		a = accuation(3.2*1e6, 0.0, -1.0);
		car3_instructions.push_back(a);
		a = accuation(3.3*1e6, 2.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(4.5*1e6, 0.0, 0.0);
	car3_instructions.push_back(a);
	a = accuation(5.5*1e6, -2.0, 0.0);
	car3_instructions.push_back(a);
	a = accuation(7.5*1e6, 0.0, 0.0);
	car3_instructions.push_back(a);
	car3.setInstructions(car3_instructions);
	if( trackCars[2] )
	{
		if(use_pf)
		{
			PF pf3;
			car3.setPF(pf3);
		}
		else if(use_mhe)
		{
			MHE mhe3;
			car3.setMHE(mhe3);
		}
		else if(use_ckf)
		{
			CKF ckf3;
			car3.setCKF(ckf3);
		}
		else if(use_iekf)
		{
			IEKF iekf3;
			car3.setIEKF(iekf3);
		}
		else if(use_ekf)
		{
			EKF ekf3;
			car3.setEKF(ekf3);
		}
		else
		{
			UKF ukf3;
			car3.setUKF(ukf3);
		}
	}
	traffic.push_back(car3);

	lidar = new Lidar(traffic,0);

	// render environment
	renderHighway(0,viewer);
	egoCar.render(viewer);
	car1.render(viewer);
	car2.render(viewer);
	car3.render(viewer);
}

void stepHighway(double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer)
	{

		if(visualize_pcd)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud = tools.loadPcd("../src/sensors/data/pcd/highway_"+std::to_string(timestamp)+".pcd");
			renderPointCloud(viewer, trafficCloud, "trafficCloud", Color((float)184/256,(float)223/256,(float)252/256));
		}
		

		// render highway environment with poles
		renderHighway(egoVelocity*timestamp/1e6, viewer);
		egoCar.render(viewer);
		
		for (int i = 0; i < traffic.size(); i++)
		{
			traffic[i].move((double)1/frame_per_sec, timestamp);
			if(!visualize_pcd)
				traffic[i].render(viewer);
			// Sense surrounding cars with lidar and radar
			if(trackCars[i])
			{
				VectorXd gt(4);
				gt << traffic[i].position.x, traffic[i].position.y, traffic[i].velocity*cos(traffic[i].angle), traffic[i].velocity*sin(traffic[i].angle);
				tools.ground_truth.push_back(gt);
				per_car_ground_truth_[i].push_back(gt);
				tools.lidarSense(traffic[i], viewer, timestamp, visualize_lidar);
				per_car_ground_truth_[i].push_back(gt);
				tools.radarSense(traffic[i], egoCar, viewer, timestamp, visualize_radar);
				tools.ukfResults(traffic[i],viewer, projectedTime, projectedSteps);
				VectorXd estimate(4);
			// Get state from active filter  (EKF, IEKF, UKF, CKF, or PF)
			VectorXd x_state;
			if(traffic[i].use_ekf) {
				x_state = traffic[i].ekf.x_;
			} else if(traffic[i].use_mhe) {
				x_state = traffic[i].mhe.x_;
			} else if(traffic[i].use_iekf) {
				x_state = traffic[i].iekf.x_;
			} else if(traffic[i].use_ckf) {
				x_state = traffic[i].ckf.x_;
			} else if(traffic[i].use_pf) {
				x_state = traffic[i].pf.x_;
			} else {
				x_state = traffic[i].ukf.x_;
			}
			double v  = x_state(2);
    		double yaw = x_state(3);
    		double v1 = cos(yaw)*v;
    		double v2 = sin(yaw)*v;
			estimate << x_state[0], x_state[1], v1, v2;
			tools.estimations.push_back(estimate);
	
			}
		}
		viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
		VectorXd rmse = tools.CalculateRMSE(tools.estimations, tools.ground_truth);
		viewer->addText(" X: "+std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
		viewer->addText(" Y: "+std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
		viewer->addText("Vx: "	+std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
		viewer->addText("Vy: "	+std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");
		
		// Print RMSE to console
		if((int)timestamp % 1000000 == 0) // Print every second
		{
			std::cout << "RMSE at " << timestamp/1e6 << "s: X=" << rmse[0] 
			          << " Y=" << rmse[1] << " Vx=" << rmse[2] << " Vy=" << rmse[3] << std::endl;
		}

		if(timestamp > 1.0e6)
		{

			if(rmse[0] > rmseThreshold[0])
			{
				rmseFailLog[0] = rmse[0];
				pass = false;
			}
			if(rmse[1] > rmseThreshold[1])
			{
				rmseFailLog[1] = rmse[1];
				pass = false;
			}
			if(rmse[2] > rmseThreshold[2])
			{
				rmseFailLog[2] = rmse[2];
				pass = false;
			}
			if(rmse[3] > rmseThreshold[3])
			{
				rmseFailLog[3] = rmse[3];
				pass = false;
			}
		}
		if(!pass)
		{
			viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
			if(rmseFailLog[0] > 0)
				viewer->addText(" X: "+std::to_string(rmseFailLog[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
			if(rmseFailLog[1] > 0)
				viewer->addText(" Y: "+std::to_string(rmseFailLog[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
			if(rmseFailLog[2] > 0)
				viewer->addText("Vx: "+std::to_string(rmseFailLog[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
			if(rmseFailLog[3] > 0)
				viewer->addText("Vy: "+std::to_string(rmseFailLog[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
		}
		
	}

	void reportRTSResults()
	{
		if ((!use_ekf_rts && !use_ukf_rts && !use_ukf_fixed_lag) || rts_reported)
		{
			return;
		}

		rts_reported = true;
		if (use_ukf_fixed_lag) {
			std::cout << "\nFixed-lag smoothing summary" << std::endl;
		} else {
			std::cout << "\nRTS smoothing summary" << std::endl;
		}

		for (int i = 0; i < traffic.size(); ++i)
		{
			if (!trackCars[i])
			{
				continue;
			}

			std::vector<VectorXd> filtered_estimations;
			std::vector<VectorXd> smoothed_estimations;

			if (use_ekf_rts) {
				if (!traffic[i].use_ekf) {
					continue;
				}

				const auto& history = traffic[i].ekf.GetStepHistory();
				if (history.empty() || per_car_ground_truth_[i].size() != history.size()) {
					std::cout << traffic[i].name << ": RTS skipped due to history size mismatch" << std::endl;
					continue;
				}

				filtered_estimations.reserve(history.size());
				for (const auto& step : history) {
					VectorXd estimate(4);
					double v = step.x_filtered(2);
					double yaw = step.x_filtered(3);
					estimate << step.x_filtered(0), step.x_filtered(1), v * cos(yaw), v * sin(yaw);
					filtered_estimations.push_back(estimate);
				}

				auto smoothed_states = ekf_rts_smoother.SmoothFromHistory(history);
				smoothed_estimations.reserve(smoothed_states.size());
				for (const auto& state : smoothed_states) {
					VectorXd estimate(4);
					double v = state.x(2);
					double yaw = state.x(3);
					estimate << state.x(0), state.x(1), v * cos(yaw), v * sin(yaw);
					smoothed_estimations.push_back(estimate);
				}
			} else if (use_ukf_rts || use_ukf_fixed_lag) {
				if (traffic[i].use_ekf || traffic[i].use_iekf || traffic[i].use_ckf ||
				    traffic[i].use_pf || traffic[i].use_mhe) {
					continue;
				}

				const auto& history = traffic[i].ukf.GetStepHistory();
				if (history.empty() || per_car_ground_truth_[i].size() != history.size()) {
					std::cout << traffic[i].name << ": UKF RTS skipped due to history size mismatch" << std::endl;
					continue;
				}

				filtered_estimations.reserve(history.size());
				for (const auto& step : history) {
					VectorXd estimate(4);
					double v = step.x_filtered(2);
					double yaw = step.x_filtered(3);
					estimate << step.x_filtered(0), step.x_filtered(1), v * cos(yaw), v * sin(yaw);
					filtered_estimations.push_back(estimate);
				}

				std::vector<UKFRTSStateEstimate> smoothed_states;
				if (use_ukf_fixed_lag) {
					smoothed_states =
						ukf_fixed_lag_smoother.SmoothFromHistory(history, fixed_lag_steps);
				} else {
					smoothed_states = ukf_rts_smoother.SmoothFromHistory(history);
				}
				smoothed_estimations.reserve(smoothed_states.size());
				for (const auto& state : smoothed_states) {
					VectorXd estimate(4);
					double v = state.x(2);
					double yaw = state.x(3);
					estimate << state.x(0), state.x(1), v * cos(yaw), v * sin(yaw);
					smoothed_estimations.push_back(estimate);
				}
			}

			VectorXd filtered_rmse = tools.CalculateRMSE(filtered_estimations, per_car_ground_truth_[i]);
			VectorXd smoothed_rmse = tools.CalculateRMSE(smoothed_estimations, per_car_ground_truth_[i]);

			std::cout << traffic[i].name
			          << " filtered RMSE: X=" << filtered_rmse(0)
			          << " Y=" << filtered_rmse(1)
			          << " Vx=" << filtered_rmse(2)
			          << " Vy=" << filtered_rmse(3) << std::endl;
			std::cout << traffic[i].name
			          << " smoothed RMSE: X=" << smoothed_rmse(0)
			          << " Y=" << smoothed_rmse(1)
			          << " Vx=" << smoothed_rmse(2)
			          << " Vy=" << smoothed_rmse(3) << std::endl;
		}
	}
	
};
