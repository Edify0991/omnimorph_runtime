/*
onnxruntime:
https://blog.csdn.net/yangyu0515/article/details/142093965
https://blog.csdn.net/m0_57254760/article/details/138304321
*/
#include "rl_master/RL_controller.h"
#include <fstream>

std::ofstream rl_debug_file;

RL_controller::RL_controller()
{
    robot = RobotState::create();
    if (!robot)
    {
        throw std::runtime_error("Failed to create RobotState object!");
    }
}

void RL_controller::RL_controller_Init()
{
    robot->RobotState_Init();
    cmd.Cmd_Init();
    filter_window = 5;
    left_knee_target_now = 0.0;
    right_knee_target_now = 0.0;
    filter_range = 0.01;
    use_filter_range = true;
    vel_threshold = 0.01;
    policy_index = 0;
    stand_flag = std::vector<float>(1, 0.0);

    // create the ONNX Runtime environment object env and set the logging level to WARNING
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "RL_controller");

    // create the ONNX Runtime session options object: session_options
    Ort::SessionOptions session_options;

    // create a default configured allocator instance that allocates and frees memory
    Ort::AllocatorWithDefaultOptions allocator;

    onnx_session_walk = new Ort::Session(env, robot->sim2realCfg.policy_path.c_str(), session_options);
    // std::cout << "ONNX model path: " << robot->standSim2RealCfg.policy_path << std::endl;
    /*-------------------------------------onnx RL stand policy file-------------------------------------*/
    onnx_session_stand = new Ort::Session(env, robot->standSim2RealCfg.policy_path.c_str(), session_options);

    /*-------------------------------------onnx RL walk policy file-------------------------------------*/
    // get model input information
    input_nodes_num_walk = onnx_session_walk->GetInputCount();
    for (int i = 0; i < input_nodes_num_walk; ++i)
    {
        auto input_name = onnx_session_walk->GetInputNameAllocated(i, allocator);
        input_node_names_walk.push_back(input_name.get());
    }

    // get model output information
    output_nodes_num_walk = onnx_session_walk->GetOutputCount();
    for (int i = 0; i < output_nodes_num_walk; ++i)
    {
        auto output_name = onnx_session_walk->GetOutputNameAllocated(i, allocator);
        output_node_names_walk.push_back(output_name.get());
    }

    // get the name of the first input and output node
    input_name_walk = {input_node_names_walk[0].c_str()};
    output_name_walk = {output_node_names_walk[0].c_str()};

    /*-------------------------------------onnx RL stand policy file-------------------------------------*/
    // get model input information
    input_nodes_num_stand = onnx_session_stand->GetInputCount();
    for (int i = 0; i < input_nodes_num_stand; ++i)
    {
        auto input_name = onnx_session_stand->GetInputNameAllocated(i, allocator);
        input_node_names_stand.push_back(input_name.get());
    }

    // get model output information
    output_nodes_num_stand = onnx_session_stand->GetOutputCount();
    for (int i = 0; i < output_nodes_num_stand; ++i)
    {
        auto output_name = onnx_session_stand->GetOutputNameAllocated(i, allocator);
        output_node_names_stand.push_back(output_name.get());
    }

    // get the name of the first input and output node
    input_name_stand = {input_node_names_stand[0].c_str()};
    output_name_stand = {output_node_names_stand[0].c_str()};

    action = std::vector<float>(robot->sim2realCfg.action_dim, 0.0);
    joint_target_q = std::vector<float>(robot->sim2realCfg.action_dim, 0.0);
    joint_target_torque = std::vector<float>(robot->sim2realCfg.action_dim, 0.0);

    // initialize the obs data queue
    obs = std::vector<float>(robot->sim2realCfg.obs_dim, 0.0);
    for (int i = 0; i < robot->sim2realCfg.obs_stack_N; ++i)
    {
        obs_deque.push_back(std::vector<float>(robot->sim2realCfg.obs_dim, 0.0));
    }

    // calculate the start_time of the phase
    start_time = std::chrono::high_resolution_clock::now();

    if (robot->sim2realCfg.save_data_flag)
    {
        rl_debug_file.open(robot->sim2realCfg.data_path + "_rl_controller_data.txt");
        std::cout << "RL Controller debug file: " << robot->sim2realCfg.data_path + "_rl_controller_data.txt" << std::endl;
    }

    // underlying control instruction flag bit shared memory
    rlSolverShm = new SharedMemory(sharedDataConfig.FILE_PATH, sharedDataConfig.length[11], sharedDataConfig.keyNum[11], sharedDataConfig.semFlag, sharedDataConfig.semName[11]);
    try
    {
        rlSolverShm->connect();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[SHM] connect() exception: " << e.what() << std::endl;
        return;
    }
}

// get single frame observation input
std::vector<float> RL_controller::get_robot_observation(double phase_t)
{
    std::vector<float> last_action = action;
    std::vector<float> q = robot->joint_q;
    std::vector<float> dq = robot->joint_dq;
    std::vector<float> w = robot->base_ang_vel;
    std::vector<float> quat = robot->base_quat;
    std::vector<float> rpy = quaternion_to_euler_array(quat);

    // std::cout << rpy[0] << "  " << rpy[1] << "  " << rpy[2] << std::endl;

    if (robot->sim2realCfg.save_data_flag && rl_debug_file.is_open())
    {
        rl_debug_file << "w: " << w[0] << ", " << w[1] << ", " << w[2] << std::endl;
        rl_debug_file << "rpy: " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl;
    }

    std::vector<float> obs(robot->sim2realCfg.obs_dim, 0.0f);

    /*  spatial structure of observation in network
        obs = torch.cat((
            command_input,  # 5 = 2D(sin cos) + 3D(vel_x, vel_y, aug_vel_yaw)
            q,    # 12D joint position
            dq,  # 12D joint velocity
            self.actions,   # 12D last action
            self.base_ang_vel * self.obs_scales.ang_vel,  # 3  # doubleing base angular velocity
            self.base_euler_xyz * self.obs_scales.quat,  # 3  # doubleing base orientation
        ), dim=-1)
    */
    // the first 5 bits of obs are reference tracks and instructions
    obs[0] = std::sin(2 * M_PI * phase_t / robot->sim2realCfg.cycle_time);
    obs[1] = std::cos(2 * M_PI * phase_t / robot->sim2realCfg.cycle_time);
    obs[2] = cmd.vx * robot->sim2realCfg.scales.lin_vel;
    obs[3] = cmd.vy * robot->sim2realCfg.scales.lin_vel;
    obs[4] = cmd.dyaw * robot->sim2realCfg.scales.ang_vel;

    for (int i = 0; i < 12; ++i)
    {
        obs[5 + i] = (q[i] - robot->default_angle[i]) * robot->sim2realCfg.scales.dof_pos; // joint position
        obs[17 + i] = dq[i] * robot->sim2realCfg.scales.dof_vel;                           // joint velocity
        obs[29 + i] = last_action[i];                                                      // last action
    }

    for (int i = 0; i < 3; ++i)
    {
        obs[41 + i] = w[i] * robot->sim2realCfg.scales.ang_vel; // doubleing base angular velocity
        obs[44 + i] = rpy[i] * robot->sim2realCfg.scales.quat;  // doubleing base orientation
    }

    for (auto &value : obs)
    {
        value = std::clamp(value, -robot->sim2realCfg.clip_observations, robot->sim2realCfg.clip_observations);
    }

    this->obs = obs;
    return obs;
}

// run the policy
std::vector<float> RL_controller::run_policy(std::deque<std::vector<float>> *obs_deque_ptr)
{
    if (!obs_deque_ptr)
    {
        obs_deque_ptr = &obs_deque;
    }

    // onnx_session_stand.Run input_tensor
    std::vector<float> obs_np;
    for (const auto &obs : *obs_deque_ptr)
    {
        obs_np.insert(obs_np.end(), obs.begin(), obs.end());
    }

    // Create input tensor object from obs_np values
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(obs_np.size())};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, obs_np.data(), obs_np.size(), input_shape.data(), input_shape.size());

    std::vector<float> action(robot->sim2realCfg.action_dim, 0.0);
    std::vector<Ort::Value> output_tensor; // output tensor

    if (std::abs(cmd.vx) + std::abs(cmd.vy) < -10000)
    { // stand policy
        policy_index = 0;
        robot->default_angle = robot->default_angle_stand;

        auto StartTime = std::chrono::high_resolution_clock::now();
        output_tensor = onnx_session_stand->Run(Ort::RunOptions{nullptr}, input_name_stand.data(), &input_tensor, 1, output_name_stand.data(), output_name_stand.size());
        auto run_time = std::chrono::high_resolution_clock::now() - StartTime;

        // std::cout << "------onnx_session_stand---------" << std::endl;

        // Extract the output tensor data
        float *float_array = output_tensor[0].GetTensorMutableData<float>();
        for (size_t i = 0; i < action.size(); ++i)
        {
            action[i] = float_array[i];
        }

        for (auto &value : action)
        {
            value = std::clamp(value, -robot->standSim2RealCfg.clip_actions, robot->standSim2RealCfg.clip_actions);
        }
        // filter the action
        for (size_t i = 0; i < action.size(); ++i)
        {
            action[i] = (1 - robot->standSim2RealCfg.action_filter) * action[i] + robot->standSim2RealCfg.action_filter * this->action[i];
        }
    }
    else
    { // walk policy
        policy_index = 1;
        robot->default_angle = robot->default_angle_walk;

        auto StartTime = std::chrono::high_resolution_clock::now();
        output_tensor = onnx_session_walk->Run(Ort::RunOptions{nullptr}, input_name_walk.data(), &input_tensor, 1, output_name_walk.data(), output_name_walk.size());
        auto run_time = std::chrono::high_resolution_clock::now() - StartTime;
        // std::cout << "------onnx_session_walk---------" << std::endl;

        // Extract the output tensor data
        float *float_array = output_tensor[0].GetTensorMutableData<float>();
        for (size_t i = 0; i < action.size(); ++i)
        {
            action[i] = static_cast<float>(float_array[i]);
        }

        for (auto &value : action)
        {
            value = std::clamp(value, -robot->sim2realCfg.clip_actions, robot->sim2realCfg.clip_actions);
        }
        // filter the action
        for (size_t i = 0; i < action.size(); ++i)
        {
            action[i] = (1 - robot->sim2realCfg.action_filter) * action[i] + robot->sim2realCfg.action_filter * this->action[i];
        }
    }

    this->action = action;
    return action;
}

// update observation queue 鍫嗗彔15甯х殑瑙傛祴鍊间綔涓鸿緭鍏?
std::deque<std::vector<float>> RL_controller::update_obs_deque(const std::vector<float> &obs)
{
    obs_deque.push_back(obs);
    obs_deque.pop_front();
    return obs_deque;
}

// PD controller calculates torque
std::vector<float> RL_controller::pd_control(const std::vector<float> &target_q, const std::vector<float> &kp, const std::vector<float> &target_dq, const std::vector<float> &kd)
{
    std::vector<float> q = robot->joint_q;
    std::vector<float> dq = robot->joint_dq;
    std::vector<float> torque(robot->sim2realCfg.action_dim, 0.0f);

    for (size_t i = 0; i < robot->sim2realCfg.action_dim; ++i)
    {
        torque[i] = (target_q[i] - q[i]) * kp[i] + (target_dq[i] - dq[i]) * kd[i];
    }

    return torque;
}

// get joint target torque
std::vector<float> RL_controller::get_joint_target_torque(const std::vector<float> &target_q)
{
    std::vector<float> target_dq(robot->sim2realCfg.action_dim, 0.0f);
    // for (int i = 0 ; i < 12 ; i++) {
    //     // 杈撳嚭kp
    //     std::cout << "i.kp:" << robot->sim2realCfg.kps[i] << std::endl;
    // }
    std::vector<float> target_tau = pd_control(target_q, robot->sim2realCfg.kps, target_dq, robot->sim2realCfg.kds);

    // limit the torque
    float hip_roll_limit = robot->sim2realCfg.robotCfg.motor_torque_limit["hip_roll_joint"];
    float hip_yaw_limit = robot->sim2realCfg.robotCfg.motor_torque_limit["hip_yaw_joint"];
    float ankle_limit = robot->sim2realCfg.robotCfg.motor_torque_limit["ankle_left_motor"];
    float knee_limit = robot->sim2realCfg.robotCfg.motor_torque_limit["knee_joint"];

    // 瀹氫箟鍏宠妭闄愬埗鏄犲皠
    auto apply_joint_limits = [&](int index, int ref_index)
    {
        if (ref_index == 0 || ref_index == 2)
        {
            target_tau[index] = std::clamp(target_tau[index], -hip_roll_limit, hip_roll_limit);
        }
        else if (ref_index == 1)
        {
            target_tau[index] = std::clamp(target_tau[index], -hip_yaw_limit, hip_yaw_limit);
        }
        else if (ref_index == 3)
        {
            target_tau[index] = std::clamp(target_tau[index], -knee_limit, knee_limit);
        }
        else if (ref_index == 4 || ref_index == 5)
        {
            target_tau[index] = std::clamp(target_tau[index], -ankle_limit, ankle_limit);
        }
    };

    // 搴旂敤闄愬埗鍒版墍鏈夊叧鑺?
    for (int i = 0; i < 12; ++i)
    {
        apply_joint_limits(i, i % 6);
    }

    joint_target_torque = target_tau;
    return target_tau;
}

// get joint target q
std::vector<float> RL_controller::get_joint_target_q(const std::vector<float> &action)
{
    std::vector<float> target_q(robot->sim2realCfg.action_dim, 0.0f);

    // restrict action
    for (size_t i = 0; i < action.size(); ++i)
    {
        target_q[i] = robot->default_angle[i] + action[i] * robot->sim2realCfg.action_scale;
    }
    rl_debug_file << "action: " << action[3] << ", " << action[9] << std::endl;

    joint_target_q = target_q;
    return target_q;
}
