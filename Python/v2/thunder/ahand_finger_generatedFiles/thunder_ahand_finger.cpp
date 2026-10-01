#include "thunder_ahand_finger.h"
#include "ahand_finger_gen.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

// --------------------------- //
// ----- Python bindings ----- //
// --------------------------- //
namespace py = pybind11;
PYBIND11_MODULE(thunder_ahand_finger_py, m) {
	py::class_<thunder_ahand_finger>(m, "thunder_ahand_finger")
		.def(py::init<>())

		.def_readonly("Dl_order", &thunder_ahand_finger::Dl_order, "Order of the link friction model")
		.def_readonly("STD_PAR_LINK", &thunder_ahand_finger::STD_PAR_LINK, "Standard number of dynamic parameters per link")
		.def_readonly("jointsType", &thunder_ahand_finger::jointsType, "Type of joints")
		.def_readonly("numJoints", &thunder_ahand_finger::numJoints, "Number of joints")
		.def_readonly("r1", &thunder_ahand_finger::r1, "Index of first revolute joint")
		.def_readonly("r2", &thunder_ahand_finger::r2, "Index of sequent revolute joint not aligned with r1")

		.def("get_d3q", &thunder_ahand_finger::get_d3q, "Get parameter: d3q")
		.def("get_d4q", &thunder_ahand_finger::get_d4q, "Get parameter: d4q")
		.def("get_ddq", &thunder_ahand_finger::get_ddq, "Get parameter: ddq")
		.def("get_ddqr", &thunder_ahand_finger::get_ddqr, "Get parameter: ddqr")
		.def("get_dq", &thunder_ahand_finger::get_dq, "Get parameter: dq")
		.def("get_dqr", &thunder_ahand_finger::get_dqr, "Get parameter: dqr")
		.def("get_par_DYN", &thunder_ahand_finger::get_par_DYN, "Get parameter: par_DYN")
		.def("get_par_Dl", &thunder_ahand_finger::get_par_Dl, "Get parameter: par_Dl")
		.def("get_par_Ia", &thunder_ahand_finger::get_par_Ia, "Get parameter: par_Ia")
		.def("get_par_Ln2EE", &thunder_ahand_finger::get_par_Ln2EE, "Get parameter: par_Ln2EE")
		.def("get_par_REG", &thunder_ahand_finger::get_par_REG, "Get parameter: par_REG")
		.def("get_par_REG_red", &thunder_ahand_finger::get_par_REG_red, "Get parameter: par_REG_red")
		.def("get_par_gravity", &thunder_ahand_finger::get_par_gravity, "Get parameter: par_gravity")
		.def("get_par_world2L0", &thunder_ahand_finger::get_par_world2L0, "Get parameter: par_world2L0")
		.def("get_q", &thunder_ahand_finger::get_q, "Get parameter: q")
		.def("get_w", &thunder_ahand_finger::get_w, "Get parameter: w")
		.def("set_d3q", &thunder_ahand_finger::set_d3q, "Set parameter: d3q", py::arg("value"))
		.def("set_d4q", &thunder_ahand_finger::set_d4q, "Set parameter: d4q", py::arg("value"))
		.def("set_ddq", &thunder_ahand_finger::set_ddq, "Set parameter: ddq", py::arg("value"))
		.def("set_ddqr", &thunder_ahand_finger::set_ddqr, "Set parameter: ddqr", py::arg("value"))
		.def("set_dq", &thunder_ahand_finger::set_dq, "Set parameter: dq", py::arg("value"))
		.def("set_dqr", &thunder_ahand_finger::set_dqr, "Set parameter: dqr", py::arg("value"))
		.def("set_par_DYN", &thunder_ahand_finger::set_par_DYN, "Set parameter: par_DYN", py::arg("value"))
		.def("set_par_Dl", &thunder_ahand_finger::set_par_Dl, "Set parameter: par_Dl", py::arg("value"))
		.def("set_par_Ia", &thunder_ahand_finger::set_par_Ia, "Set parameter: par_Ia", py::arg("value"))
		.def("set_par_Ln2EE", &thunder_ahand_finger::set_par_Ln2EE, "Set parameter: par_Ln2EE", py::arg("value"))
		.def("set_par_REG", &thunder_ahand_finger::set_par_REG, "Set parameter: par_REG", py::arg("value"))
		.def("set_par_REG_red", &thunder_ahand_finger::set_par_REG_red, "Set parameter: par_REG_red", py::arg("value"))
		.def("set_par_gravity", &thunder_ahand_finger::set_par_gravity, "Set parameter: par_gravity", py::arg("value"))
		.def("set_par_world2L0", &thunder_ahand_finger::set_par_world2L0, "Set parameter: par_world2L0", py::arg("value"))
		.def("set_q", &thunder_ahand_finger::set_q, "Set parameter: q", py::arg("value"))
		.def("set_w", &thunder_ahand_finger::set_w, "Set parameter: w", py::arg("value"))

		.def("get_C", &thunder_ahand_finger::get_C, "Manipulator Coriolis matrix")
		.def("get_C_ddot", &thunder_ahand_finger::get_C_ddot, "Second time derivative of the Coriolis matrix")
		.def("get_C_dot", &thunder_ahand_finger::get_C_dot, "Time derivative of the Coriolis matrix")
		.def("get_C_std", &thunder_ahand_finger::get_C_std, "Classic formulation of the manipulator Coriolis matrix")
		.def("get_Dl1", &thunder_ahand_finger::get_Dl1, "SEA manipulator link damping, order 1")
		.def("get_Dl2", &thunder_ahand_finger::get_Dl2, "SEA manipulator link damping, order 2")
		.def("get_G", &thunder_ahand_finger::get_G, "Manipulator gravity terms")
		.def("get_G_ddot", &thunder_ahand_finger::get_G_ddot, "Second time derivative of the gravity vector")
		.def("get_G_dot", &thunder_ahand_finger::get_G_dot, "Time derivative of the gravity vector")
		.def("get_Ia", &thunder_ahand_finger::get_Ia, "Manipulator motor inertia torque")
		.def("get_J_1", &thunder_ahand_finger::get_J_1, "Jacobian of frame 1")
		.def("get_J_2", &thunder_ahand_finger::get_J_2, "Jacobian of frame 2")
		.def("get_J_3", &thunder_ahand_finger::get_J_3, "Jacobian of frame 3")
		.def("get_J_4", &thunder_ahand_finger::get_J_4, "Jacobian of frame 4")
		.def("get_J_5", &thunder_ahand_finger::get_J_5, "Jacobian of frame 5")
		.def("get_J_cm_1", &thunder_ahand_finger::get_J_cm_1, "Jacobian of center of mass of link 1")
		.def("get_J_cm_2", &thunder_ahand_finger::get_J_cm_2, "Jacobian of center of mass of link 2")
		.def("get_J_cm_3", &thunder_ahand_finger::get_J_cm_3, "Jacobian of center of mass of link 3")
		.def("get_J_cm_4", &thunder_ahand_finger::get_J_cm_4, "Jacobian of center of mass of link 4")
		.def("get_J_ee", &thunder_ahand_finger::get_J_ee, "Jacobian of the end-effector")
		.def("get_J_ee_ddot", &thunder_ahand_finger::get_J_ee_ddot, "Time second derivative of jacobian matrix")
		.def("get_J_ee_dot", &thunder_ahand_finger::get_J_ee_dot, "Time derivative of jacobian matrix")
		.def("get_J_ee_pinv", &thunder_ahand_finger::get_J_ee_pinv, "Pseudo-Inverse of jacobian matrix")
		.def("get_M", &thunder_ahand_finger::get_M, "Manipulator mass matrix")
		.def("get_M_ddot", &thunder_ahand_finger::get_M_ddot, "Second time derivative of the mass matrix")
		.def("get_M_dot", &thunder_ahand_finger::get_M_dot, "Time derivative of the mass matrix")
		.def("get_T_0", &thunder_ahand_finger::get_T_0, "relative transformation from frame world to base")
		.def("get_T_1", &thunder_ahand_finger::get_T_1, "relative transformation from frame0to frame 1")
		.def("get_T_2", &thunder_ahand_finger::get_T_2, "relative transformation from frame1to frame 2")
		.def("get_T_3", &thunder_ahand_finger::get_T_3, "relative transformation from frame2to frame 3")
		.def("get_T_4", &thunder_ahand_finger::get_T_4, "relative transformation from frame3to frame 4")
		.def("get_T_JOINT_P", &thunder_ahand_finger::get_T_JOINT_P, "Template transformation of general prismatic joint R")
		.def("get_T_JOINT_R", &thunder_ahand_finger::get_T_JOINT_R, "Template transformation of general rotoidal joint R")
		.def("get_T_w_0", &thunder_ahand_finger::get_T_w_0, "absolute transformation from frame world to base")
		.def("get_T_w_1", &thunder_ahand_finger::get_T_w_1, "absolute transformation from frame base to frame 1")
		.def("get_T_w_2", &thunder_ahand_finger::get_T_w_2, "absolute transformation from frame base to frame 2")
		.def("get_T_w_3", &thunder_ahand_finger::get_T_w_3, "absolute transformation from frame base to frame 3")
		.def("get_T_w_4", &thunder_ahand_finger::get_T_w_4, "absolute transformation from frame base to frame 4")
		.def("get_T_w_5", &thunder_ahand_finger::get_T_w_5, "absolute transformation from frame base to end_effector")
		.def("get_T_w_ee", &thunder_ahand_finger::get_T_w_ee, "absolute transformation from frame 0 to end_effector")
		.def("get_Yr", &thunder_ahand_finger::get_Yr, "Manipulator regressor matrix")
		.def("get_Yr_red", &thunder_ahand_finger::get_Yr_red, "Regressor defined w.r.t the set of base iniertial parameters.")
		.def("get_beta", &thunder_ahand_finger::get_beta, "linear relationship between full dyn parameters and the reduced set. beta s.t. par_red = beta*par.")
		.def("get_dl", &thunder_ahand_finger::get_dl, "Manipulator link friction")
		.def("get_dyn2reg", &thunder_ahand_finger::get_dyn2reg, "Conversion from dynamic to regressor parameters")
		.def("get_par_KIN", &thunder_ahand_finger::get_par_KIN, "Internal kinematic parameters.")
		.def("get_reg2dyn", &thunder_ahand_finger::get_reg2dyn, "Conversion from regressor to dynamic parameters")
		.def("get_reg2red", &thunder_ahand_finger::get_reg2red, "Conversion from regressor to base inertial parameters.")
		.def("get_reg_C", &thunder_ahand_finger::get_reg_C, "Regressor matrix of term C*dqr")
		.def("get_reg_C_red", &thunder_ahand_finger::get_reg_C_red, "Regressor of centripetal defined w.r.t the set of base iniertial parameters.")
		.def("get_reg_G", &thunder_ahand_finger::get_reg_G, "Regressor matrix of term G")
		.def("get_reg_G_red", &thunder_ahand_finger::get_reg_G_red, "Regressor of gravity defined w.r.t the set of base iniertial parameters.")
		.def("get_reg_Ia", &thunder_ahand_finger::get_reg_Ia, "Regressor matrix of the motor inertia")
		.def("get_reg_JTw", &thunder_ahand_finger::get_reg_JTw, "Regressor matrix of the quantity J^T*w")
		.def("get_reg_Jdq", &thunder_ahand_finger::get_reg_Jdq, "Regressor matrix of the quantity J*dq")
		.def("get_reg_M", &thunder_ahand_finger::get_reg_M, "Regressor matrix of term M*ddqr")
		.def("get_reg_M_red", &thunder_ahand_finger::get_reg_M_red, "Regressor of masses defined w.r.t the set of base iniertial parameters.")
		.def("get_reg_dl", &thunder_ahand_finger::get_reg_dl, "Regressor matrix of the link friction");
}

thunder_ahand_finger::thunder_ahand_finger(){}

// get the parameter: d3q
Vector<double,4> thunder_ahand_finger::get_d3q() {return d3q;}
// get the parameter: d4q
Vector<double,4> thunder_ahand_finger::get_d4q() {return d4q;}
// get the parameter: ddq
Vector<double,4> thunder_ahand_finger::get_ddq() {return ddq;}
// get the parameter: ddqr
Vector<double,4> thunder_ahand_finger::get_ddqr() {return ddqr;}
// get the parameter: dq
Vector<double,4> thunder_ahand_finger::get_dq() {return dq;}
// get the parameter: dqr
Vector<double,4> thunder_ahand_finger::get_dqr() {return dqr;}
// get the parameter: par_DYN
Vector<double,40> thunder_ahand_finger::get_par_DYN() {return par_DYN;}
// get the parameter: par_Dl
Vector<double,8> thunder_ahand_finger::get_par_Dl() {return par_Dl;}
// get the parameter: par_Ia
Vector<double,4> thunder_ahand_finger::get_par_Ia() {return par_Ia;}
// get the parameter: par_Ln2EE
Vector<double,6> thunder_ahand_finger::get_par_Ln2EE() {return par_Ln2EE;}
// get the parameter: par_REG
Vector<double,40> thunder_ahand_finger::get_par_REG() {return par_REG;}
// get the parameter: par_REG_red
Vector<double,26> thunder_ahand_finger::get_par_REG_red() {return par_REG_red;}
// get the parameter: par_gravity
Vector<double,3> thunder_ahand_finger::get_par_gravity() {return par_gravity;}
// get the parameter: par_world2L0
Vector<double,6> thunder_ahand_finger::get_par_world2L0() {return par_world2L0;}
// get the parameter: q
Vector<double,4> thunder_ahand_finger::get_q() {return q;}
// get the parameter: w
Vector<double,6> thunder_ahand_finger::get_w() {return w;}
// set the parameter: d3q
void thunder_ahand_finger::set_d3q(Vector<double,4> value) {d3q = value;}
// set the parameter: d4q
void thunder_ahand_finger::set_d4q(Vector<double,4> value) {d4q = value;}
// set the parameter: ddq
void thunder_ahand_finger::set_ddq(Vector<double,4> value) {ddq = value;}
// set the parameter: ddqr
void thunder_ahand_finger::set_ddqr(Vector<double,4> value) {ddqr = value;}
// set the parameter: dq
void thunder_ahand_finger::set_dq(Vector<double,4> value) {dq = value;}
// set the parameter: dqr
void thunder_ahand_finger::set_dqr(Vector<double,4> value) {dqr = value;}
// set the parameter: par_DYN
void thunder_ahand_finger::set_par_DYN(Vector<double,40> value) {par_DYN = value;}
// set the parameter: par_Dl
void thunder_ahand_finger::set_par_Dl(Vector<double,8> value) {par_Dl = value;}
// set the parameter: par_Ia
void thunder_ahand_finger::set_par_Ia(Vector<double,4> value) {par_Ia = value;}
// set the parameter: par_Ln2EE
void thunder_ahand_finger::set_par_Ln2EE(Vector<double,6> value) {par_Ln2EE = value;}
// set the parameter: par_REG
void thunder_ahand_finger::set_par_REG(Vector<double,40> value) {par_REG = value;}
// set the parameter: par_REG_red
void thunder_ahand_finger::set_par_REG_red(Vector<double,26> value) {par_REG_red = value;}
// set the parameter: par_gravity
void thunder_ahand_finger::set_par_gravity(Vector<double,3> value) {par_gravity = value;}
// set the parameter: par_world2L0
void thunder_ahand_finger::set_par_world2L0(Vector<double,6> value) {par_world2L0 = value;}
// set the parameter: q
void thunder_ahand_finger::set_q(Vector<double,4> value) {q = value;}
// set the parameter: w
void thunder_ahand_finger::set_w(Vector<double,6> value) {w = value;}

// Save parameters to file
int thunder_ahand_finger::save_par(string par_file, vector<string> par_list){
	YAML::Node yamlFile;

	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d3q"))){
		vector<double> d3q_vect(d3q.data(), d3q.data() + 4);
		yamlFile["d3q"] = d3q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d4q"))){
		vector<double> d4q_vect(d4q.data(), d4q.data() + 4);
		yamlFile["d4q"] = d4q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddq"))){
		vector<double> ddq_vect(ddq.data(), ddq.data() + 4);
		yamlFile["ddq"] = ddq_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddqr"))){
		vector<double> ddqr_vect(ddqr.data(), ddqr.data() + 4);
		yamlFile["ddqr"] = ddqr_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dq"))){
		vector<double> dq_vect(dq.data(), dq.data() + 4);
		yamlFile["dq"] = dq_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dqr"))){
		vector<double> dqr_vect(dqr.data(), dqr.data() + 4);
		yamlFile["dqr"] = dqr_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_DYN"))){
		vector<double> par_DYN_vect(par_DYN.data(), par_DYN.data() + 40);
		yamlFile["par_DYN"] = par_DYN_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Dl"))){
		vector<double> par_Dl_vect(par_Dl.data(), par_Dl.data() + 8);
		yamlFile["par_Dl"] = par_Dl_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Ia"))){
		vector<double> par_Ia_vect(par_Ia.data(), par_Ia.data() + 4);
		yamlFile["par_Ia"] = par_Ia_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Ln2EE"))){
		vector<double> par_Ln2EE_vect(par_Ln2EE.data(), par_Ln2EE.data() + 6);
		yamlFile["par_Ln2EE"] = par_Ln2EE_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG"))){
		vector<double> par_REG_vect(par_REG.data(), par_REG.data() + 40);
		yamlFile["par_REG"] = par_REG_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG_red"))){
		vector<double> par_REG_red_vect(par_REG_red.data(), par_REG_red.data() + 26);
		yamlFile["par_REG_red"] = par_REG_red_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_gravity"))){
		vector<double> par_gravity_vect(par_gravity.data(), par_gravity.data() + 3);
		yamlFile["par_gravity"] = par_gravity_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_world2L0"))){
		vector<double> par_world2L0_vect(par_world2L0.data(), par_world2L0.data() + 6);
		yamlFile["par_world2L0"] = par_world2L0_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "q"))){
		vector<double> q_vect(q.data(), q.data() + 4);
		yamlFile["q"] = q_vect;
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "w"))){
		vector<double> w_vect(w.data(), w.data() + 6);
		yamlFile["w"] = w_vect;
	}

	try {
		YAML::Emitter emitter;
		emitter.SetIndent(2);
		emitter.SetSeqFormat(YAML::Flow);
		emitter << yamlFile << YAML::Newline;
		std::ofstream fout(par_file);
		fout << emitter.c_str();
		fout.close();
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while generating YAML: " << e.what() << std::endl;
		return 0;
	}
	return 1;
}


// Load parameters from file
int thunder_ahand_finger::load_par(string par_file, vector<string> par_list){
	YAML::Node yamlFile;
	try {
		yamlFile = YAML::LoadFile(par_file);

	} catch (const YAML::Exception& e) {
		std::cerr << "Error while loading parameters: " << e.what() << std::endl;
		return 0;
	}

	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d3q"))){
		if (yamlFile["d3q"]){
			vector<double> vec(yamlFile["d3q"].as<vector<double>>());
			d3q = Eigen::Map<Vector<double,4>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: d3q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "d4q"))){
		if (yamlFile["d4q"]){
			vector<double> vec(yamlFile["d4q"].as<vector<double>>());
			d4q = Eigen::Map<Vector<double,4>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: d4q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddq"))){
		if (yamlFile["ddq"]){
			vector<double> vec(yamlFile["ddq"].as<vector<double>>());
			ddq = Eigen::Map<Vector<double,4>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: ddq not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "ddqr"))){
		if (yamlFile["ddqr"]){
			vector<double> vec(yamlFile["ddqr"].as<vector<double>>());
			ddqr = Eigen::Map<Vector<double,4>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: ddqr not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dq"))){
		if (yamlFile["dq"]){
			vector<double> vec(yamlFile["dq"].as<vector<double>>());
			dq = Eigen::Map<Vector<double,4>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: dq not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "dqr"))){
		if (yamlFile["dqr"]){
			vector<double> vec(yamlFile["dqr"].as<vector<double>>());
			dqr = Eigen::Map<Vector<double,4>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: dqr not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_DYN"))){
		if (yamlFile["par_DYN"]){
			vector<double> vec(yamlFile["par_DYN"].as<vector<double>>());
			par_DYN = Eigen::Map<Vector<double,40>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_DYN not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Dl"))){
		if (yamlFile["par_Dl"]){
			vector<double> vec(yamlFile["par_Dl"].as<vector<double>>());
			par_Dl = Eigen::Map<Vector<double,8>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_Dl not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Ia"))){
		if (yamlFile["par_Ia"]){
			vector<double> vec(yamlFile["par_Ia"].as<vector<double>>());
			par_Ia = Eigen::Map<Vector<double,4>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_Ia not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_Ln2EE"))){
		if (yamlFile["par_Ln2EE"]){
			vector<double> vec(yamlFile["par_Ln2EE"].as<vector<double>>());
			par_Ln2EE = Eigen::Map<Vector<double,6>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_Ln2EE not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG"))){
		if (yamlFile["par_REG"]){
			vector<double> vec(yamlFile["par_REG"].as<vector<double>>());
			par_REG = Eigen::Map<Vector<double,40>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_REG not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_REG_red"))){
		if (yamlFile["par_REG_red"]){
			vector<double> vec(yamlFile["par_REG_red"].as<vector<double>>());
			par_REG_red = Eigen::Map<Vector<double,26>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_REG_red not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_gravity"))){
		if (yamlFile["par_gravity"]){
			vector<double> vec(yamlFile["par_gravity"].as<vector<double>>());
			par_gravity = Eigen::Map<Vector<double,3>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_gravity not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "par_world2L0"))){
		if (yamlFile["par_world2L0"]){
			vector<double> vec(yamlFile["par_world2L0"].as<vector<double>>());
			par_world2L0 = Eigen::Map<Vector<double,6>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: par_world2L0 not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "q"))){
		if (yamlFile["q"]){
			vector<double> vec(yamlFile["q"].as<vector<double>>());
			q = Eigen::Map<Vector<double,4>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: q not found!" << std::endl;
			return 0;
		}
	}
	if ((par_list.size()==0)||(std::count(par_list.begin(), par_list.end(), "w"))){
		if (yamlFile["w"]){
			vector<double> vec(yamlFile["w"].as<vector<double>>());
			w = Eigen::Map<Vector<double,6>>(vec.data(), vec.size());
		} else {
			std::cerr << "Error while loading parameters: w not found!" << std::endl;
			return 0;
		}
	}

	return 1;
}


// Manipulator Coriolis matrix
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_C() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_C_fun_SZ_IW];
	thread_local double p4[ahand_finger_C_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_C_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Second time derivative of the Coriolis matrix
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_C_ddot() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_C_ddot_fun_SZ_IW];
	thread_local double p4[ahand_finger_C_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), d3q.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_C_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Time derivative of the Coriolis matrix
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_C_dot() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_C_dot_fun_SZ_IW];
	thread_local double p4[ahand_finger_C_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_C_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Classic formulation of the manipulator Coriolis matrix
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_C_std() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_C_std_fun_SZ_IW];
	thread_local double p4[ahand_finger_C_std_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_C_std_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// SEA manipulator link damping, order 1
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_Dl1() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_Dl1_fun_SZ_IW];
	thread_local double p4[ahand_finger_Dl1_fun_SZ_W];
	const double* input_[] = {par_Dl.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_Dl1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// SEA manipulator link damping, order 2
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_Dl2() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_Dl2_fun_SZ_IW];
	thread_local double p4[ahand_finger_Dl2_fun_SZ_W];
	const double* input_[] = {par_Dl.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_Dl2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Manipulator gravity terms
Eigen::Matrix<double,4,1> thunder_ahand_finger::get_G() {
	thread_local double buffer[4];
	thread_local long long p3[ahand_finger_G_fun_SZ_IW];
	thread_local double p4[ahand_finger_G_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_gravity.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_G_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,1>>(buffer);
}

// Second time derivative of the gravity vector
Eigen::Matrix<double,4,1> thunder_ahand_finger::get_G_ddot() {
	thread_local double buffer[4];
	thread_local long long p3[ahand_finger_G_ddot_fun_SZ_IW];
	thread_local double p4[ahand_finger_G_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), par_world2L0.data(), par_gravity.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_G_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,1>>(buffer);
}

// Time derivative of the gravity vector
Eigen::Matrix<double,4,1> thunder_ahand_finger::get_G_dot() {
	thread_local double buffer[4];
	thread_local long long p3[ahand_finger_G_dot_fun_SZ_IW];
	thread_local double p4[ahand_finger_G_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_world2L0.data(), par_gravity.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_G_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,1>>(buffer);
}

// Manipulator motor inertia torque
Eigen::Matrix<double,4,1> thunder_ahand_finger::get_Ia() {
	thread_local double buffer[4];
	thread_local long long p3[ahand_finger_Ia_fun_SZ_IW];
	thread_local double p4[ahand_finger_Ia_fun_SZ_W];
	const double* input_[] = {ddq.data(), par_Ia.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_Ia_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,1>>(buffer);
}

// Jacobian of frame 1
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_1() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_1_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_1_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Jacobian of frame 2
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_2() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_2_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_2_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Jacobian of frame 3
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_3() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_3_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_3_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Jacobian of frame 4
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_4() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_4_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_4_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Jacobian of frame 5
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_5() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_5_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_5_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Jacobian of center of mass of link 1
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_cm_1() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_cm_1_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_cm_1_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_cm_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Jacobian of center of mass of link 2
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_cm_2() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_cm_2_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_cm_2_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_cm_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Jacobian of center of mass of link 3
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_cm_3() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_cm_3_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_cm_3_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_cm_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Jacobian of center of mass of link 4
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_cm_4() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_cm_4_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_cm_4_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_cm_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Jacobian of the end-effector
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_ee() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_ee_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_ee_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_ee_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Time second derivative of jacobian matrix
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_ee_ddot() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_ee_ddot_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_ee_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), par_world2L0.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_ee_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Time derivative of jacobian matrix
Eigen::Matrix<double,6,4> thunder_ahand_finger::get_J_ee_dot() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_ee_dot_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_ee_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_world2L0.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_ee_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,4>>(buffer);
}

// Pseudo-Inverse of jacobian matrix
Eigen::Matrix<double,4,6> thunder_ahand_finger::get_J_ee_pinv() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_J_ee_pinv_fun_SZ_IW];
	thread_local double p4[ahand_finger_J_ee_pinv_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_J_ee_pinv_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,6>>(buffer);
}

// Manipulator mass matrix
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_M() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_M_fun_SZ_IW];
	thread_local double p4[ahand_finger_M_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_M_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Second time derivative of the mass matrix
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_M_ddot() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_M_ddot_fun_SZ_IW];
	thread_local double p4[ahand_finger_M_ddot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), ddq.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_M_ddot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Time derivative of the mass matrix
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_M_dot() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_M_dot_fun_SZ_IW];
	thread_local double p4[ahand_finger_M_dot_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_world2L0.data(), par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_M_dot_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame world to base
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_0() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_0_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_0_fun_SZ_W];
	const double* input_[] = {par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame0to frame 1
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_1() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_1_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_1_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame1to frame 2
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_2() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_2_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_2_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame2to frame 3
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_3() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_3_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_3_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// relative transformation from frame3to frame 4
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_4() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_4_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_4_fun_SZ_W];
	const double* input_[] = {q.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of general prismatic joint R
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_JOINT_P(Vector<double,1> q_joint, Vector<double,3> axes) {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_JOINT_P_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_JOINT_P_fun_SZ_W];
	const double* input_[] = {q_joint.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_JOINT_P_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Template transformation of general rotoidal joint R
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_JOINT_R(Vector<double,1> q_joint, Vector<double,3> axes) {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_JOINT_R_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_JOINT_R_fun_SZ_W];
	const double* input_[] = {q_joint.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_JOINT_R_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame world to base
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_w_0() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_w_0_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_w_0_fun_SZ_W];
	const double* input_[] = {par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_w_0_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 1
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_w_1() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_w_1_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_w_1_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_w_1_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 2
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_w_2() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_w_2_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_w_2_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_w_2_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 3
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_w_3() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_w_3_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_w_3_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_w_3_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to frame 4
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_w_4() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_w_4_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_w_4_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_w_4_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame base to end_effector
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_w_5() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_w_5_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_w_5_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_w_5_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// absolute transformation from frame 0 to end_effector
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_T_w_ee() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_T_w_ee_fun_SZ_IW];
	thread_local double p4[ahand_finger_T_w_ee_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_T_w_ee_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Manipulator regressor matrix
Eigen::Matrix<double,4,40> thunder_ahand_finger::get_Yr() {
	thread_local double buffer[160];
	thread_local long long p3[ahand_finger_Yr_fun_SZ_IW];
	thread_local double p4[ahand_finger_Yr_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data(), par_world2L0.data(), par_gravity.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_Yr_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,40>>(buffer);
}

// Regressor defined w.r.t the set of base iniertial parameters.
Eigen::Matrix<double,4,26> thunder_ahand_finger::get_Yr_red() {
	thread_local double buffer[104];
	thread_local long long p3[ahand_finger_Yr_red_fun_SZ_IW];
	thread_local double p4[ahand_finger_Yr_red_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data(), par_world2L0.data(), par_gravity.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_Yr_red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,26>>(buffer);
}

// linear relationship between full dyn parameters and the reduced set. beta s.t. par_red = beta*par.
Eigen::Matrix<double,26,44> thunder_ahand_finger::get_beta() {
	thread_local double buffer[1144];
	thread_local long long p3[ahand_finger_beta_fun_SZ_IW];
	thread_local double p4[ahand_finger_beta_fun_SZ_W];
	const double** input_ = nullptr;
	double* output_[] = {buffer};
	int check = ahand_finger_beta_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,26,44>>(buffer);
}

// Manipulator link friction
Eigen::Matrix<double,4,1> thunder_ahand_finger::get_dl() {
	thread_local double buffer[4];
	thread_local long long p3[ahand_finger_dl_fun_SZ_IW];
	thread_local double p4[ahand_finger_dl_fun_SZ_W];
	const double* input_[] = {dq.data(), par_Dl.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_dl_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,1>>(buffer);
}

// Conversion from dynamic to regressor parameters
Eigen::Matrix<double,40,1> thunder_ahand_finger::get_dyn2reg() {
	thread_local double buffer[40];
	thread_local long long p3[ahand_finger_dyn2reg_fun_SZ_IW];
	thread_local double p4[ahand_finger_dyn2reg_fun_SZ_W];
	const double* input_[] = {par_DYN.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_dyn2reg_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,40,1>>(buffer);
}

// Internal kinematic parameters.
Eigen::Matrix<double,24,1> thunder_ahand_finger::get_par_KIN() {
	thread_local double buffer[24];
	thread_local long long p3[ahand_finger_par_KIN_fun_SZ_IW];
	thread_local double p4[ahand_finger_par_KIN_fun_SZ_W];
	const double** input_ = nullptr;
	double* output_[] = {buffer};
	int check = ahand_finger_par_KIN_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,24,1>>(buffer);
}

// Conversion from regressor to dynamic parameters
Eigen::Matrix<double,40,1> thunder_ahand_finger::get_reg2dyn() {
	thread_local double buffer[40];
	thread_local long long p3[ahand_finger_reg2dyn_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg2dyn_fun_SZ_W];
	const double* input_[] = {par_REG.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg2dyn_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,40,1>>(buffer);
}

// Conversion from regressor to base inertial parameters.
Eigen::Matrix<double,26,1> thunder_ahand_finger::get_reg2red() {
	thread_local double buffer[26];
	thread_local long long p3[ahand_finger_reg2red_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg2red_fun_SZ_W];
	const double* input_[] = {par_REG.data(), par_Ia.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg2red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,26,1>>(buffer);
}

// Regressor matrix of term C*dqr
Eigen::Matrix<double,4,40> thunder_ahand_finger::get_reg_C() {
	thread_local double buffer[160];
	thread_local long long p3[ahand_finger_reg_C_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_C_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_C_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,40>>(buffer);
}

// Regressor of centripetal defined w.r.t the set of base iniertial parameters.
Eigen::Matrix<double,4,24> thunder_ahand_finger::get_reg_C_red() {
	thread_local double buffer[96];
	thread_local long long p3[ahand_finger_reg_C_red_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_C_red_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), dqr.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_C_red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,24>>(buffer);
}

// Regressor matrix of term G
Eigen::Matrix<double,4,40> thunder_ahand_finger::get_reg_G() {
	thread_local double buffer[160];
	thread_local long long p3[ahand_finger_reg_G_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_G_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_gravity.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_G_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,40>>(buffer);
}

// Regressor of gravity defined w.r.t the set of base iniertial parameters.
Eigen::Matrix<double,4,24> thunder_ahand_finger::get_reg_G_red() {
	thread_local double buffer[96];
	thread_local long long p3[ahand_finger_reg_G_red_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_G_red_fun_SZ_W];
	const double* input_[] = {q.data(), par_world2L0.data(), par_gravity.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_G_red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,24>>(buffer);
}

// Regressor matrix of the motor inertia
Eigen::Matrix<double,4,4> thunder_ahand_finger::get_reg_Ia() {
	thread_local double buffer[16];
	thread_local long long p3[ahand_finger_reg_Ia_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_Ia_fun_SZ_W];
	const double* input_[] = {ddq.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_Ia_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,4>>(buffer);
}

// Regressor matrix of the quantity J^T*w
Eigen::Matrix<double,4,12> thunder_ahand_finger::get_reg_JTw() {
	thread_local double buffer[48];
	thread_local long long p3[ahand_finger_reg_JTw_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_JTw_fun_SZ_W];
	const double* input_[] = {q.data(), w.data(), par_world2L0.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_JTw_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,12>>(buffer);
}

// Regressor matrix of the quantity J*dq
Eigen::Matrix<double,6,12> thunder_ahand_finger::get_reg_Jdq() {
	thread_local double buffer[72];
	thread_local long long p3[ahand_finger_reg_Jdq_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_Jdq_fun_SZ_W];
	const double* input_[] = {q.data(), dq.data(), par_world2L0.data(), par_Ln2EE.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_Jdq_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,6,12>>(buffer);
}

// Regressor matrix of term M*ddqr
Eigen::Matrix<double,4,40> thunder_ahand_finger::get_reg_M() {
	thread_local double buffer[160];
	thread_local long long p3[ahand_finger_reg_M_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_M_fun_SZ_W];
	const double* input_[] = {q.data(), ddqr.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_M_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,40>>(buffer);
}

// Regressor of masses defined w.r.t the set of base iniertial parameters.
Eigen::Matrix<double,4,24> thunder_ahand_finger::get_reg_M_red() {
	thread_local double buffer[96];
	thread_local long long p3[ahand_finger_reg_M_red_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_M_red_fun_SZ_W];
	const double* input_[] = {q.data(), ddqr.data(), par_world2L0.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_M_red_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,24>>(buffer);
}

// Regressor matrix of the link friction
Eigen::Matrix<double,4,8> thunder_ahand_finger::get_reg_dl() {
	thread_local double buffer[32];
	thread_local long long p3[ahand_finger_reg_dl_fun_SZ_IW];
	thread_local double p4[ahand_finger_reg_dl_fun_SZ_W];
	const double* input_[] = {dq.data()};
	double* output_[] = {buffer};
	int check = ahand_finger_reg_dl_fun(input_, output_, p3, p4, 0);
	return Eigen::Map<Eigen::Matrix<double,4,8>>(buffer);
}

