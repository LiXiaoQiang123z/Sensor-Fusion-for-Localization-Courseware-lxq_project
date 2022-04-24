// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

//
// TODO: implement analytic Jacobians for LOAM residuals in this file
//

#include <eigen3/Eigen/Dense>

//
// TODO: Sophus is ready to use if you have a good undestanding of Lie algebra.
//
#include <sophus/so3.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#define auto_or_analytic 1 // 0:auto

// Eigen::Matrix3d skew(Eigen::Vector3d& mat_in); //反对称矩阵声明
template <typename T>
Eigen::Matrix<T, 3, 3> skew(Eigen::Matrix<T, 3, 1> &mat_in)
{
	Eigen::Matrix<T, 3, 3> skew_mat;
	skew_mat.setZero();
	skew_mat(0, 1) = -mat_in(2);
	skew_mat(0, 2) = mat_in(1);
	skew_mat(1, 2) = -mat_in(0);
	skew_mat(1, 0) = mat_in(2);
	skew_mat(2, 0) = -mat_in(1);
	skew_mat(2, 1) = mat_in(0);
	return skew_mat;
};
/**
 * @brief Get the Transform From so3 object
 * 从so3转换成 Transform： 指数映射 ； 这里用公式推得
 * @param so3
 * @param q
 * @param t
 */
inline void getTransformFromSo3(const Eigen::Matrix<double, 3, 1> &so3, Eigen::Quaterniond &q)
{
	Eigen::Vector3d omega(so3.data());
	Eigen::Matrix3d Omega = skew(omega);

	double theta = omega.norm();
	double half_theta = 0.5 * theta;

	double imag_factor;
	double real_factor = cos(half_theta); // 实部
	if (theta < 1e-10)
	{
		double theta_sq = theta * theta;
		double theta_po4 = theta_sq * theta_sq;
		imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4; // sin theta泰勒展开把
	}
	else
	{
		double sin_half_theta = sin(half_theta);
		imag_factor = sin_half_theta / theta;
	}

	q = Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());

	// Eigen::Matrix3d J;
	// if (theta<1e-10)
	// {
	//     J = q.matrix();
	// }
	// else
	// {
	//     Eigen::Matrix3d Omega2 = Omega*Omega;
	//     J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(pow(theta,3))*Omega2);
	// }

	// t = J*upsilon;
};

/********************************[ori]***********************************/
#if auto_or_analytic == 0
// struct LidarEdgeFactor
// {
// 	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
// 					Eigen::Vector3d last_point_b_, double s_)
// 		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

// 	template <typename T>
// 	bool operator()(const T *q, const T *t, T *residual) const
// 	{

// 		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
// 		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
// 		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

// 		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
// 		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
// 		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
// 		q_last_curr = q_identity.slerp(T(s), q_last_curr);
// 		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

// 		Eigen::Matrix<T, 3, 1> lp;
// 		lp = q_last_curr * cp + t_last_curr;

// 		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
// 		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

// 		residual[0] = nu.x() / de.norm();
// 		residual[1] = nu.y() / de.norm();
// 		residual[2] = nu.z() / de.norm();

// 		return true;
// 	}

// 	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
// 									   const Eigen::Vector3d last_point_b_, const double s_)
// 	{
// 		return (new ceres::AutoDiffCostFunction<
// 				LidarEdgeFactor, 3, 4, 3>(
// 			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
// 	}

// 	Eigen::Vector3d curr_point, last_point_a, last_point_b;
// 	double s;
// };

// struct LidarPlaneFactor
// {
// 	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
// 					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
// 		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
// 		  last_point_m(last_point_m_), s(s_)
// 	{
// 		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
// 		ljm_norm.normalize();
// 	}

// 	template <typename T>
// 	bool operator()(const T *q, const T *t, T *residual) const
// 	{

// 		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
// 		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
// 		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
// 		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
// 		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

// 		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
// 		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
// 		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
// 		q_last_curr = q_identity.slerp(T(s), q_last_curr);
// 		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

// 		Eigen::Matrix<T, 3, 1> lp;
// 		lp = q_last_curr * cp + t_last_curr;

// 		residual[0] = (lp - lpj).dot(ljm);

// 		return true;
// 	}

// 	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
// 									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
// 									   const double s_)
// 	{
// 		return (new ceres::AutoDiffCostFunction<
// 				LidarPlaneFactor, 1, 4, 3>(
// 			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
// 	}

// 	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
// 	Eigen::Vector3d ljm_norm;
// 	double s;
// };

/******************************[start]*************************************/
#elif auto_or_analytic == 1
/**
 * @brief  针对ａｌｏａｍ 问题: 
 * 1. problem.AddParameterBlock(para_q, 4, q_parameterization);  四元数重构问题，所以需要自定义 LocalParameterization 
 * 自定义 LocalParameterization ： 1.1 plus增量的计算 + 1.2 雅克比矩阵的计算【可以参考Floam(注意：重构的是q、t)】
 * [单独的四元数，好像调用现成的就行啊 。。。 白写？]
 * 2. 残差块-手写雅克比
 */

class PoseQuaternionParameterization : public ceres::LocalParameterization // 与基类命名一致，直接复制即可
{
public:
	PoseQuaternionParameterization() {}
	virtual ~PoseQuaternionParameterization() {}
	/**
	 * @brief 重载的plus函数
	 *
	 * @param x 更新前的四元数
	 * @param delta 用旋转矢量表示的增量
	 * @param x_plus_delta 更新后的四元数
	 */
	virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
	{
			// const double norm_delta =
			// 	sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
			// if (norm_delta > 0.0)
			// {
			// 	const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
			// 	double q_delta[4];
			// 	q_delta[0] = cos(norm_delta);
			// 	q_delta[1] = sin_delta_by_delta * delta[0];
			// 	q_delta[2] = sin_delta_by_delta * delta[1];
			// 	q_delta[3] = sin_delta_by_delta * delta[2];
			// 	ceres::QuaternionProduct(q_delta, x, x_plus_delta);
			// }
			// else
			// {
			// 	for (int i = 0; i < 4; ++i)
			// 	{
			// 		x_plus_delta[i] = x[i];
			// 	}
			// }

			// 利用sophus库
			Eigen::Map<const Eigen::Quaterniond> quater(x);		//   待更新的四元数
			Eigen::Map<const Eigen::Vector3d> delta_so3(delta); //    delta 值,使用流形 so3 更新
			Eigen::Quaterniond delta_quater = Sophus::SO3d::exp(delta_so3).unit_quaternion(); //   so3 转换位 delta_p  四元数
			Eigen::Map<Eigen::Quaterniond> quter_plus(x_plus_delta); //   更新后的四元数
			// 旋转更新公式
			quter_plus = (delta_quater * quater).normalized();

			// 手动计算求解
			// Eigen::Quaterniond delta_q;
			// getTransformFromSo3(Eigen::Map<const Eigen::Matrix<double, 3, 1>>(delta), delta_q);
			// Eigen::Map<const Eigen::Quaterniond> quater(x);			  // 优化前的四元数
			// Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta); // 优化后的四元数
			// quater_plus = (delta_q * quater).normalized();

			return true;
	};

	/**
	 * @brief J(4x3) = dq/dx = d[qw,qx,qy,qz]^T / d[x,y,z]
	 * TODO: 
	ceres::QuaternionParameterization ：内部存储顺序为`(w,x,y,z)`
	ceres::EigenQuaternionParameterization：内部存储顺序为`(x,y,z,w)`
	Eigen::Quaternion(w,x,y,z)：内部存储顺序为`(x,y,z,w)` 顺序不同
	 * @param x
	 * @param jacobian
	 */
	virtual bool ComputeJacobian(const double* x, double* jacobian) const
	{
			// jacobian[0] = -x[1]; jacobian[1]  = -x[2]; jacobian[2]  = -x[3];  // NOLINT
			// jacobian[3] =  x[0]; jacobian[4]  =  x[3]; jacobian[5]  = -x[2];  // NOLINT
			// jacobian[6] = -x[3]; jacobian[7]  =  x[0]; jacobian[8]  =  x[1];  // NOLINT
			// jacobian[9] =  x[2]; jacobian[10] = -x[1]; jacobian[11] =  x[0];  // NOLINT

			Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian); // 优化后的四元数
			(j.topRows(3)).setIdentity();
			(j.bottomRows(1)).setZero();

		return true;
	};

	// GlobalSize 返回值为4，即四元数本身的实际维数。由于在内部优化时，ceres采用的是旋转矢量，维数为3，因此LocalSize()的返回值为3。
	// GlobalSize 就是表示他真正的维数是一个4维的 || 【如果是q+t=7】
	virtual int GlobalSize() const { return 4; }
	// LocalSize是告诉Ceres他表示的东西是一个三维的 ||【如果是q+t=６】
	virtual int LocalSize() const { return 3; }
};

// 2.直接在F-loam上面进行更改
// 线特征
class LidarEdgeFactorClass : public ceres::SizedCostFunction<1, 4,3> {
	public:

		LidarEdgeFactorClass(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_,  double s_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_),s(s_){

		};
		// virtual ~LidarEdgeFactorClass() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
			Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]); //   存放 w  x y z
			Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);
			Eigen::Vector3d lp; //   line point
			Eigen::Vector3d lp_r;
			lp_r = q_last_curr * curr_point;
			lp = q_last_curr * curr_point + t_last_curr; //   new point
			Eigen::Vector3d nu = (lp - last_point_b).cross(lp - last_point_a); // 修改为pi-pb x pi-pa
			Eigen::Vector3d de = last_point_a - last_point_b;

			residuals[0] = nu.norm() / de.norm(); //  线残差

			if (jacobians != NULL)
			{
                if (jacobians[0]  !=  NULL)
                {
					Eigen::Matrix3d skew_de = skew(de);

					//  J_so3_Rotation
					Eigen::Matrix3d   skew_lp_r  =  skew(lp_r);
					Eigen::Matrix3d    dp_by_dr;
					dp_by_dr.block<3,3>(0,0)  =  -skew_lp_r;
					Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_r(jacobians[0]);
					J_so3_r.setZero();
					J_so3_r.block<1,3>(0,0)  =   nu.transpose()* skew_de * dp_by_dr / (de.norm()*nu.norm());

					//  J_so3_Translation
					Eigen::Matrix3d  dp_by_dt;
					(dp_by_dt.block<3,3>(0,0)).setIdentity();
					Eigen::Map<Eigen::Matrix<double,  1,  3,  Eigen::RowMajor>> J_so3_t(jacobians[1]);
					J_so3_t.setZero();
					J_so3_t.block<1,3>(0,0)  =   nu.transpose()  *  skew_de * dp_by_dt / (de.norm()*nu.norm());
                }
        }
        return  true;
		};
		Eigen::Vector3d curr_point;
		Eigen::Vector3d last_point_a;
		Eigen::Vector3d last_point_b;

		double s;
};


// 面特征
class LidarPlaneFactorClass : public ceres::SizedCostFunction<1, 4,3> {
	public:
		LidarPlaneFactorClass(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
							  Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
			: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_), last_point_m(last_point_m_), s(s_){};
		// virtual ~LidarPlaneFactorClass() {}

	    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{

                // 叉乘运算， j,l,m 三个但构成的平行四边面积(摸)和该面的单位法向量(方向)
				Eigen::Vector3d ljm_norm = (last_point_l - last_point_j).cross(last_point_m - last_point_j); // 【l-j x m-j】
				ljm_norm.normalize(); //  单位法向量=等价除以自身绝对值

				Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
				Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);

				Eigen::Vector3d lp;								 // “从当前阵的当前点” 经过转换矩阵转换到“上一阵的同线束激光点”
				Eigen::Vector3d lp_r = q_last_curr * curr_point; //  for compute jacobian o rotation  L: dp_dr
				lp = q_last_curr * curr_point + t_last_curr;

				// 残差函数
				double phi1 = (lp - last_point_j).dot(ljm_norm); //点积得到的是长度=标量
				residuals[0]  =   std::fabs(phi1);

				if(jacobians != NULL)
                {
                        if(jacobians[0] != NULL)
                        {
							phi1 = phi1 / residuals[0]; // 标量没有转置的概念一说
							//  Rotation
							Eigen::Matrix3d skew_lp_r = skew(lp_r);
							Eigen::Matrix3d dp_dr;
							dp_dr.block<3, 3>(0, 0) = -skew_lp_r;
							Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_r(jacobians[0]);
							J_so3_r.setZero();
							J_so3_r.block<1, 3>(0, 0) = phi1 * ljm_norm.transpose() * (dp_dr);

							Eigen::Matrix3d dp_by_dt;
							(dp_by_dt.block<3, 3>(0, 0)).setIdentity();
							Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_so3_t(jacobians[1]);
							J_so3_t.block<1, 3>(0, 0) = phi1 * ljm_norm.transpose() * dp_by_dt;
						}
                }
                return  true;
		};

		Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
		double s;
};

#endif
/*********************************[SE3-摘自网上博客：未来得及实验【TODO:挖坑】]*******************************************/
// class SophusLidarEdgeFactor : public ceres::SizedCostFunction<1, 6> {
// public:

//         // Eigen::Vector3d curr_point, last_point_a, last_point_b;
// 		// LidarEdgeFactorClass(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
// 		// 						 const Eigen::Vector3d last_point_b_, const double s_)
// 		// 	: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

//     SophusLidarEdgeFactor(
//         const Eigen::Vector3d cur_point, 
//         const Eigen::Vector3d last_point_a, 
//         const Eigen::Vector3d last_point_b) : 
//             cur_point_(cur_point), 
//     		last_point_a_(last_point_a), 
//     		last_point_b_(last_point_b) {}
    
//     virtual ~SophusLidarEdgeFactor() {}

//     virtual bool Evaluate(double const * const * parameters, 
//                           double *residuals, double **jacobians) const {
//         Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(parameters[0]);
//         Sophus::SE3d T = Sophus::SE3d::exp(lie);

//         Eigen::Vector3d lp = T * cur_point_;
//         Eigen::Vector3d de = last_point_a_ - last_point_b_;
//         Eigen::Vector3d nu = (lp - last_point_b_).cross(lp - last_point_a_);
//         residuals[0] = nu.norm() / de.norm();

//         if (jacobians != nullptr) {
//             if (jacobians[0] != nullptr) {
//                 Eigen::Matrix3d lp_hat = Sophus::SO3d::hat(lp);
//                 Eigen::Matrix<double, 3, 6> dp_dse3;
//                 (dp_dse3.block<3, 3>(0, 0)).setIdentity();
//                 dp_dse3.block<3, 3>(0, 3) = -lp_hat;

//                 Eigen::Map<Eigen::Matrix<double, 1, 6>> J_se3(jacobians[0]);
//                 J_se3.setZero();
                
//                 Eigen::Matrix<double, 1, 3> de_dp = 
//                     (nu / (de.norm() * nu.norm())).transpose() * Sophus::SO3d::hat(de);
//                 J_se3.block<1, 6>(0, 0) = de_dp * dp_dse3;
//             }
//         }
//         return true;
//     }

//     Eigen::Vector3d cur_point_;
//     Eigen::Vector3d last_point_a_;
//     Eigen::Vector3d last_point_b_;
// };

// class SophusLidarPlaneFactor : public ceres::SizedCostFunction<1, 6> {
// public:

//         // LidarPlaneFactorClass(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
// 		// 			 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
// 		// : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),last_point_m(last_point_m_), s(s_){}

//     SophusLidarPlaneFactor(
//         Eigen::Vector3d curr_point, 
//         Eigen::Vector3d last_point_j,
//         Eigen::Vector3d last_point_l, 
//         Eigen::Vector3d last_point_m) : 
//         curr_point_(curr_point), 
//         last_point_j_(last_point_j), 
//         last_point_l_(last_point_l), 
//         last_point_m_(last_point_m) { }

//     virtual ~SophusLidarPlaneFactor() {}
    
//     virtual bool Evaluate(double const *const *parameters, 
//                           double *residuals, 
//                           double **jacobians) const {
//         Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(parameters[0]);
//         Sophus::SE3d T = Sophus::SE3d::exp(lie);
//         Eigen::Vector3d lpi = T * curr_point_;
//         Eigen::Vector3d pipj = lpi - last_point_j_;
//         Eigen::Vector3d ljm_norm = 
//             (last_point_l_ - last_point_j_).cross(last_point_m_ - last_point_j_);
//         ljm_norm.normalize();

//         residuals[0] = pipj.dot(ljm_norm);
//         if (jacobians != nullptr) {
//             if (jacobians[0] != nullptr) {
//                 Eigen::Matrix3d lp_hat = Sophus::SO3d::hat(lpi);
//                 Eigen::Matrix<double, 3, 6> dp_dse3;
//                 (dp_dse3.block<3, 3>(0, 0)).setIdentity();
//                 dp_dse3.block<3, 3>(0, 3) = -lp_hat;

//                 Eigen::Matrix<double, 1, 6> J = ljm_norm.transpose() * dp_dse3;
//                 jacobians[0][0] = J(0);
//                 jacobians[0][1] = J(1);
//                 jacobians[0][2] = J(2);
//                 jacobians[0][3] = J(3);
//                 jacobians[0][4] = J(4);
//                 jacobians[0][5] = J(5);
//             }
//         }
//         return true;
//     }
//     Eigen::Vector3d curr_point_, last_point_j_, last_point_l_, last_point_m_;
    
// };

// class PoseSE3Parameterization : public ceres::LocalParameterization {
// public:
//     PoseSE3Parameterization() {}

//     virtual ~PoseSE3Parameterization() {}

//     virtual bool Plus(const double *x, 
//                       const double *delta, 
//                       double *x_plus_delta) const {
//         Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(x);
//         Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_lie(delta);

//         Sophus::SE3d T = Sophus::SE3d::exp(lie);
//         Sophus::SE3d delta_T = Sophus::SE3d::exp(delta_lie);
//         Eigen::Matrix<double, 6, 1> x_plus_delta_lie = (delta_T * T).log();
        
//         for (size_t i = 0; i < 6; i++) {
//             x_plus_delta[i] = x_plus_delta_lie(i, 0);
//         }
//         return true;
//     }

//     virtual bool ComputeJacobian(const double *x, double *jacobian) const {
//         ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
//         return true;
//     }

//     virtual int GlobalSize() const {return Sophus::SE3d::DoF;}
//     virtual int LocalSize() const {return Sophus::SE3d::DoF;}
// };
/*********************************[end]*******************************************/
struct LidarPlaneNormFactor
{

	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 4, 3>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};


struct LidarDistanceFactor
{

	LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_) 
						: curr_point(curr_point_), closed_point(closed_point_){}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;


		residual[0] = point_w.x() - T(closed_point.x());
		residual[1] = point_w.y() - T(closed_point.y());
		residual[2] = point_w.z() - T(closed_point.z());
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarDistanceFactor, 3, 4, 3>(
			new LidarDistanceFactor(curr_point_, closed_point_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d closed_point;
};