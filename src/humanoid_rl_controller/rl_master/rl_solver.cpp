#include <iostream>
#include <atomic>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <cmath>
#include <tuple>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cstring>
#include <cstdint>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <cerrno>
#include <Eigen/Dense>
#include <csignal> // 淇″彿澶勭悊
#include <SharedMemory.hpp>
#include <sched.h> // CPU 璋冨害
#include <sys/mman.h>
#include <time.h>
#include <errno.h>
#include "rl_master/rl_cfg.h"
#include "rl_master/Ankle_Kinematics.h"
#include "rl_master/Knee_Kinematics.h"
#include "rl_master/KinConv.h"
#include "rl_master/math_tool.h"
#include "rl_master/rl_protocol.h"
#include "rl_master/solver_dds_bridge.h"
#include <deque>
#include <numeric>

constexpr long CONTROL_PERIOD_NS = 2'000'000; // 500 Hz

// 瀹氫箟鐢垫満鍙ユ焺缁撴瀯浣?
typedef struct
{
    uint8_t run_mode;   /** see @MotorRunMode */
    uint8_t motor_type; /** see @MotorType */
    uint8_t pd[2];

    union
    {
        struct
        {
            float target_speed;  /** for motor_type = MOTOR_TYPE_LINE, the unit is mm/s, otherwise rad/s */
            float target_pos;    /** for motor_type = MOTOR_TYPE_LINE, the unit is mm, otherwise rad */
            float target_torque; /** for motor_type = MOTOR_TYPE_LINE, the unit is N, otherwise N*M */
        };

        struct
        {
            float feedback_speed;  /** for motor_type = MOTOR_TYPE_LINE, the unit is mm/s, otherwise rad/s */
            float feedback_pos;    /** for motor_type = MOTOR_TYPE_LINE, the unit is mm, otherwise rad */
            float feedback_torque; /** for motor_type = MOTOR_TYPE_LINE, the unit is N, otherwise N*M */
        };
    };
} MHandle;

#define MOTOR_COUNT_MAX (30)       // 12涓數鏈?
#define INSTALLED_MOTOR_COUNT (12) // 鐜板凡缁忓畨瑁?2涓數鏈?
#define SHARED_MEM_SEM_FLAGE (0x00)

#define SHM_TARGET_PATH "/home/jc_robot/target_handle"
#define SHM_TARGET_SEM_NAME "sem_target_handle"
#define SHM_TARGET_KEY_NUM (0x01)
static SharedMemory *shm_target;

#define SHM_FEEDBACK_PATH "/home/jc_robot/feedback_handle"
#define SHM_FEEDBACK_SEM_NAME "sem_feedback_handle"
#define SHM_FEEDBACK_KEY_NUM (0x02)
static SharedMemory *shm_feedback;

MHandle gMHandle[MOTOR_COUNT_MAX];

std::atomic<bool> runFlag(true);

std::ofstream data_file;

static void app_set_all_motor_target(MHandle *motorHandle)
{
    if (!shm_target)
    {
        std::cerr << "Shared memory not initialized!" << std::endl;
        return;
    }
    shm_target->write(motorHandle, MOTOR_COUNT_MAX, 0);
}

static void app_get_all_motor_fdb(MHandle *motorHandle)
{
    if (!shm_feedback)
    {
        std::cerr << "Shared memory not initialized!" << std::endl;
        return;
    }
    shm_feedback->read(motorHandle, MOTOR_COUNT_MAX, 0);
}


void set_realtime_priority(int cpu_id = 2, int priority = 90)
{
    /* 1. 閿佷綇鍐呭瓨锛岄槻姝?page fault */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
        std::cerr << "mlockall failed: " << strerror(errno) << std::endl;
    }

    /* 2. 缁戝畾鍒版寚瀹?CPU 鏍?*/
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
    {
        std::cerr << "sched_setaffinity failed: " << strerror(errno) << std::endl;
    }

    /* 3. 璁剧疆 FIFO 瀹炴椂璋冨害 */
    struct sched_param param;
    param.sched_priority = priority; // 鎺ㄨ崘 80鈥?5锛屼笉瑕佺敤 99
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
    {
        std::cerr << "sched_setscheduler failed: " << strerror(errno) << std::endl;
    }

    /* 4. 纭 */
    std::cout << "[RT] FIFO priority=" << priority
              << " cpu=" << cpu_id << std::endl;
}

class MovingAverageFilter
{
private:
    std::deque<float> buffer;
    size_t window_size;
    float sum;

public:
    MovingAverageFilter(size_t size = 5) : window_size(size), sum(0.0)
    {
        // buffer.reserve(window_size);
    }

    float update(float new_value)
    {
        buffer.push_back(new_value);
        sum += new_value;

        if (buffer.size() > window_size)
        {
            sum -= buffer.front();
            buffer.pop_front();
        }

        return sum / buffer.size();
    }

    void reset()
    {
        buffer.clear();
        sum = 0.0;
    }
};

class RobotSolver
{
public:
    RobotSolver() = default;
    static std::unique_ptr<RobotSolver> create()
    {
        auto robotSolver = std::unique_ptr<RobotSolver>(new RobotSolver());
        if (!robotSolver->sim2realCfg.loadFromYAML(RL_CFG_PATH, "sim2real"))
        {
            std::cerr << "Failed to load Sim2Real config!" << std::endl;
            return nullptr;
        }
        else
        {
            std::cout << "Sim2Real config loaded successfully!" << std::endl;
        }
        if (!robotSolver->standSim2RealCfg.loadFromYAML(RL_CFG_PATH, "stand_sim2real"))
        {
            std::cerr << "Failed to load Stand Sim2Real config!" << std::endl;
            return nullptr;
        }
        else
        {
            std::cout << "Stand Sim2Real config loaded successfully!" << std::endl;
        }
        return robotSolver;
    }

    void robotSolver_Init()
    {
        // joint data: rad
        joint_state = std::vector<JointData>(MOTOR_COUNT_MAX, {0, 0, 0, RUN_MODE_CSP, 0, 0}); // joint state
        joint_cmd = std::vector<JointData>(MOTOR_COUNT_MAX, {0, 0, 0, RUN_MODE_CSP, 0, 0});   // joint command
        motor_state = std::vector<JointData>(MOTOR_COUNT_MAX, {0, 0, 0, RUN_MODE_CSP, 0, 0}); // motor state
        motor_cmd = std::vector<JointData>(MOTOR_COUNT_MAX, {0, 0, 0, RUN_MODE_CSP, 0, 0});   // motor command

        open_rl = 0;
        last_open_rl = 0;

        // data array
        joint_cmd_q = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        joint_cmd_dq = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        joint_cmd_tau = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        joint_state_q = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        joint_state_dq = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        joint_state_tau = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        motor_cmd_q = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        motor_cmd_dq = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        motor_cmd_tau = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        motor_state_q = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        motor_state_dq = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        motor_state_tau = std::vector<float>(MOTOR_COUNT_MAX, 0.0);
        motor_cmd_mode = std::vector<float>(MOTOR_COUNT_MAX, 0.0);

        for (int i = 0; i < INSTALLED_MOTOR_COUNT; i++)
        {
            joint_cmd[i].kp = sim2realCfg.kps[i];
            joint_cmd[i].kd = sim2realCfg.kds[i];
            joint_state[i].kp = sim2realCfg.kps[i];
            joint_state[i].kd = sim2realCfg.kds[i];
            std::cout << "jointstate kp [ " << i << "]:" << static_cast<int>(joint_state[i].kp) << std::endl;
        }

        // 鍒濆鍖栫數鏈哄叡浜唴瀛?
        shm_target = new SharedMemory(SHM_TARGET_PATH, sizeof(MHandle) * MOTOR_COUNT_MAX, SHM_TARGET_KEY_NUM, LOCK_TYPE_MUTEX, SHM_TARGET_SEM_NAME);
        shm_feedback = new SharedMemory(SHM_FEEDBACK_PATH, sizeof(MHandle) * MOTOR_COUNT_MAX, SHM_FEEDBACK_KEY_NUM, LOCK_TYPE_MUTEX, SHM_FEEDBACK_SEM_NAME);

        try
        {
            shm_target->connect();
            shm_feedback->connect();
            dds_bridge_.connect();
        }
        catch (const std::exception &e)
        {
            std::cerr << "[RL_solver] initialization exception: " << e.what() << std::endl;
            return;
        }

        // 鍒濆鍖栫數鏈虹被鍨嬮厤缃?
        initMotorTypes();

        // 鍒濆鍖栨护娉㈠櫒锛岀獥鍙ｅぇ灏忎负5
        for (auto &filter : velocity_filters)
        {
            filter = MovingAverageFilter(5);
        }
    }

    void initMotorTypes()
    {
        // 璁剧疆鐢垫満绫诲瀷锛?=鏃嬭浆鐢垫満锛?=绾挎€х數鏈?
        // 鏍规嵁鏂伴厤缃細10涓棆杞數鏈?+ 2涓嚎鎬х數鏈猴紙鑶濆叧鑺傦級
        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            if (i == 3 || i == 9)
            {                               // knee motors (right and left)
                gMHandle[i].motor_type = 1; // 绾挎€х數鏈?
            }
            else
            {                               // hip roll, yaw, pitch 鍜?ankle pitch, roll
                gMHandle[i].motor_type = 0; // 鏃嬭浆鐢垫満
            }
        }
    }

    std::vector<float> pd_control(const std::vector<float> &target_q, const std::vector<float> &target_dq)
    {
        std::vector<float> torque(INSTALLED_MOTOR_COUNT, 0.0f);

        for (size_t i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            torque[i] = (target_q[i] - joint_state[i].q) * static_cast<float>(joint_cmd[i].kp) + (target_dq[i] - joint_state[i].dq) * static_cast<float>(joint_cmd[i].kd);
        }

        return torque;
    }

    // get motor state from shared memory using MHandle approach
    void getMotorState()
    {
        /*
        鐢垫満绱㈠紩鏄犲皠锛?
        motor_state[0]: hip_motor_r_roll    -> 鏃嬭浆鐢垫満
        motor_state[1]: hip_motor_r_yaw     -> 鏃嬭浆鐢垫満
        motor_state[2]: hip_motor_r_pitch   -> 鏃嬭浆鐢垫満
        motor_state[3]: knee_motor_r        -> 绾挎€х數鏈?
        motor_state[4]: ankle_motor_rl      -> 鏃嬭浆鐢垫満
        motor_state[5]: ankle_motor_rr      -> 鏃嬭浆鐢垫満
        motor_state[6]: hip_motor_l_roll    -> 鏃嬭浆鐢垫満
        motor_state[7]: hip_motor_l_yaw     -> 鏃嬭浆鐢垫満
        motor_state[8]: hip_motor_l_pitch   -> 鏃嬭浆鐢垫満
        motor_state[9]: knee_motor_l        -> 绾挎€х數鏈?
        motor_state[10]: ankle_motor_ll     -> 鏃嬭浆鐢垫満
        motor_state[11]: ankle_motor_lr     -> 鏃嬭浆鐢垫満
        */
        // 浠庡叡浜唴瀛樿幏鍙栨墍鏈夌數鏈虹殑鍙嶉鐘舵€?
        app_get_all_motor_fdb(motor_feedback_all);

        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            motor_state[i].q = motor_feedback_all[i].feedback_pos;
            motor_state[i].dq = motor_feedback_all[i].feedback_speed;
            motor_state[i].tau = motor_feedback_all[i].feedback_torque;

            motor_state_q[i] = motor_state[i].q;
            motor_state_dq[i] = motor_state[i].dq;
            motor_state_tau[i] = motor_state[i].tau;

            // 閽堝2鍙峰拰8鍙风數鏈虹殑閫熷害妫€鏌?
            if (i == 2 || i == 8)
            {
                const float SPEED_LIMIT = 2.7f; // 2.7 rad/s

                // 鑾峰彇褰撳墠閫熷害鐨勭粷瀵瑰€?
                float current_speed = fabs(motor_state[i].dq);

                if (current_speed > SPEED_LIMIT)
                {
                    // 鎵撳嵃閿欒淇℃伅
                    std::cerr << "鉂?绱ф€ュ仠姝紒鐢垫満 #" << i << " 閫熷害瓒呴檺锛? "<< std::endl;
                    std::cerr << "   褰撳墠閫熷害: " << motor_state[i].dq << " rad/s" << std::endl;
                    std::cerr << "   闄愬埗閫熷害: " << SPEED_LIMIT << " rad/s" << std::endl;
                    std::cerr << "   瓒呭嚭: " << (current_speed - SPEED_LIMIT) << " rad/s" << std::endl;

                    // 涓柇绋嬪簭
                    // std::terminate(); // 鎴?exit(EXIT_FAILURE)
                }
            }

            // 浠呮鏌?鍙峰拰9鍙风數鏈?
            if (i == 3 || i == 9)
            {
                // 0-60mm鑼冨洿妫€鏌?
                const float Q_MIN_MM = -0.1f;
                const float Q_MAX_MM = 60.0f;

                // 鍗曚綅杞崲
                float q_mm = motor_state_q[i];

                // 鑼冨洿妫€鏌?
                if (q_mm < Q_MIN_MM || q_mm > Q_MAX_MM)
                {
                    // 鎵撳嵃璇︾粏閿欒淇℃伅
                    std::cerr << "========================================" << std::endl;
                    std::cerr << "馃毃 鐢垫満瀹夊叏闄愬埗瑙﹀彂锛佺▼搴忓嵆灏嗙粓姝€?" << std::endl;
                    std::cerr << "----------------------------------------" << std::endl;
                    std::cerr << "鏁呴殰鐢垫満: #" << i << std::endl;
                    std::cerr << "褰撳墠鍊? " << q_mm << " mm " << std::endl;
                    std::cerr << "鍏佽鑼冨洿: [" << Q_MIN_MM << " mm, "
                              << Q_MAX_MM << " mm]" << std::endl;
                    std::cerr << "瓒呭嚭闄愬埗: "
                              << (q_mm < Q_MIN_MM ? q_mm - Q_MIN_MM : q_mm - Q_MAX_MM)
                              << " mm" << std::endl;
                    std::cerr << "========================================" << std::endl;

                    // 寮哄埗缁堟绋嬪簭
                    // std::terminate(); // 鎴?exit(EXIT_FAILURE)
                }
            }
            // std::cout << "motor_state[" << i << "] q:" << motor_state[i].q << "  dq:" << motor_state[i].dq << "  tau:" << motor_state[i].tau << std::endl;
        }

        joint_state = kinConv.legMotorToJoint(motor_state);

        // store motor command data
        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            joint_state_q[i] = joint_state[i].q;
            joint_state_dq[i] = joint_state[i].dq;
            joint_state_tau[i] = joint_state[i].tau;
        }
    }

    // write motor command to shared memory using MotorHandle approach
    void sendMotorCmd()
    {
        /*
        鐢垫満绱㈠紩鏄犲皠锛?
        motor_cmd[0]: hip_motor_r_roll    -> 鏃嬭浆鐢垫満
        motor_cmd[1]: hip_motor_r_yaw     -> 鏃嬭浆鐢垫満
        motor_cmd[2]: hip_motor_r_pitch   -> 鏃嬭浆鐢垫満
        motor_cmd[3]: knee_motor_r        -> 绾挎€х數鏈?
        motor_cmd[4]: ankle_motor_rl      -> 鏃嬭浆鐢垫満
        motor_cmd[5]: ankle_motor_rr      -> 鏃嬭浆鐢垫満
        motor_cmd[6]: hip_motor_l_roll    -> 鏃嬭浆鐢垫満
        motor_cmd[7]: hip_motor_l_yaw     -> 鏃嬭浆鐢垫満
        motor_cmd[8]: hip_motor_l_pitch   -> 鏃嬭浆鐢垫満
        motor_cmd[9]: knee_motor_l        -> 绾挎€х數鏈?
        motor_cmd[10]: ankle_motor_ll     -> 鏃嬭浆鐢垫満
        motor_cmd[11]: ankle_motor_lr     -> 鏃嬭浆鐢垫満
        */
        // 鍚戝叡浜唴瀛樺彂閫佹墍鏈夌數鏈虹殑鐩爣鍊?
        memset(motor_target_all, 0, sizeof(motor_target_all));

        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            joint_cmd_q[i] = joint_cmd[i].q;
            joint_cmd_dq[i] = joint_cmd[i].dq;
            joint_cmd_tau[i] = joint_cmd[i].tau;
        }

        motor_cmd = kinConv.legJointToMotor(joint_state, joint_cmd);

        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            motor_target_all[i].target_speed = motor_cmd[i].dq;
            motor_target_all[i].target_pos = motor_cmd[i].q;
            motor_target_all[i].target_torque = motor_cmd[i].tau;
            motor_target_all[i].run_mode = static_cast<uint8_t>(joint_cmd[i].mode);
            motor_target_all[i].pd[0] = static_cast<uint8_t>(joint_cmd[i].kp);
            motor_target_all[i].pd[1] = static_cast<uint8_t>(joint_cmd[i].kd);

            // store motor command data
            motor_cmd_q[i] = motor_cmd[i].q;
            motor_cmd_dq[i] = motor_cmd[i].dq;
            motor_cmd_tau[i] = motor_cmd[i].tau;

            motor_cmd_mode[i] = static_cast<float>(joint_cmd[i].mode);
        }
        app_set_all_motor_target(motor_target_all);
        // std::cout << "------app_set_all_motor_target---------" << std::endl;

        // 鎵撳嵃璋冭瘯淇℃伅
        // std::cout << "Motor commands sent to shared memory" << std::endl;

        // std::cout << "-------------------------------------------------------------------------------" << std::endl;
        // // 鎵撳嵃鏃嬭浆鐢垫満鍛戒护锛坔ip roll, yaw, pitch 鍜?ankle pitch, roll锛?
        // std::vector<int> rotary_motors = {0, 1, 2, 4, 5, 6, 7, 8, 10, 11}; // 鍙宠吙鏃嬭浆鐢垫満 + 宸﹁吙鏃嬭浆鐢垫満
        // for (int k = 0; k < rotary_motors.size(); ++k)
        // {
        //     int motor_idx = rotary_motors[k];
        //     std::cout << "rotateMotorCmd[" << motor_idx << "] tau:" << motor_cmd[motor_idx].tau << "  pos:" << motor_cmd[motor_idx].q << std::endl;
        // }
        // // 鎵撳嵃绾挎€х數鏈哄懡浠わ紙knee motors锛?
        // for (int j = 0; j < 2; ++j)
        // {
        //     int knee_index = (j == 0) ? 3 : 9; // knee_motor_r=3, knee_motor_l=9
        //     std::cout << "lineMotorCmd[" << j << "] tau:" << motor_cmd[knee_index].tau << "  pos:" << motor_cmd[knee_index].q << std::endl;
        // }
        // std::cout << "-------------------------------------------------------------------------------" << std::endl;
    }

    // Receive policy command from DDS.
    void getRLCmd()
    {
        rl_master::RobotCommandData dds_cmd{};
        uint32_t cmd_seq = 0;
        double cmd_stamp_s = 0.0;
        if (!dds_bridge_.readLatestPolicyCommand(&dds_cmd, &cmd_seq, &cmd_stamp_s))
        {
            latest_cmd_fresh = false;
            last_open_rl = open_rl;
            open_rl = static_cast<int>(rl_master::kOpenRlDisabled);
            return;
        }

        std::vector<float> target_q(INSTALLED_MOTOR_COUNT, 0.0f);
        std::vector<float> target_dq(INSTALLED_MOTOR_COUNT, 0.0f);
        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            joint_cmd[i].q = dds_cmd.joint_target_q[static_cast<size_t>(i)];
            joint_cmd[i].dq = dds_cmd.joint_target_dq[static_cast<size_t>(i)];
            joint_cmd[i].tau = dds_cmd.joint_target_tau[static_cast<size_t>(i)];

            target_q[i] = joint_cmd[i].q;
            target_dq[i] = joint_cmd[i].dq;
            if (i == 0 || i == 1 || i == 2 || i == 6 || i == 7 || i == 8)
            {
                joint_cmd[i].mode = RUN_MODE_R1;
            }
            else
            {
                joint_cmd[i].mode = RUN_MODE_CST;
            }
        }

        std::vector<float> joint_tau = pd_control(target_q, target_dq);
        constexpr float scale = 1.0f;
        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            if (joint_cmd[i].mode == RUN_MODE_CST)
            {
                joint_cmd[i].tau = joint_tau[i] * scale;
            }
        }

        last_open_rl = open_rl;
        open_rl = static_cast<int>(dds_cmd.open_rl);

        const double now_s = rl_master::monotonicTimeSec();
        latest_cmd_fresh = true;
        if (sim2realCfg.enable_cmd_watchdog && cmd_stamp_s > 1e-6)
        {
            if (cmd_seq == last_cmd_seq_)
            {
                latest_cmd_fresh = false;
            }
            if ((now_s - cmd_stamp_s) > sim2realCfg.cmd_timeout_s)
            {
                latest_cmd_fresh = false;
            }
            if (!latest_cmd_fresh && (now_s - last_stale_warn_time_s_) > 1.0)
            {
                std::cerr << "[RL_solver] stale RL command detected. Switching to hold mode." << std::endl;
                last_stale_warn_time_s_ = now_s;
            }
            if (latest_cmd_fresh)
            {
                last_cmd_seq_ = cmd_seq;
            }
        }

        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            joint_cmd_q[i] = joint_cmd[i].q;
            joint_cmd_dq[i] = joint_cmd[i].dq;
            joint_cmd_tau[i] = joint_cmd[i].tau;
        }
    }

    // Publish current robot state to DDS.
    void sendRLState()
    {
        dds_bridge_.publishRobotState(joint_state);
    }

    std::map<std::string, std::vector<float>> get_robot_state_bag()
    {
        // 鑾峰彇鏃堕棿鎴?
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              now - start_time)
                              .count();
        float time_ms = elapsed_us / 1000.0f;

        std::map<std::string, std::vector<float>> robot_data = {
            {"timestamp", {time_ms}},
            {"joint_cmd_q", joint_cmd_q},
            {"joint_cmd_dq", joint_cmd_dq},
            {"joint_cmd_tau", joint_cmd_tau},
            {"joint_state_q", joint_state_q},
            {"joint_state_dq", joint_state_dq},
            {"joint_state_tau", joint_state_tau},
            {"motor_cmd_q", motor_cmd_q},
            {"motor_cmd_dq", motor_cmd_dq},
            {"motor_cmd_tau", motor_cmd_tau},
            {"motor_state_q", motor_state_q},
            {"motor_state_dq", motor_state_dq},
            {"motor_state_tau", motor_state_tau},
            {"motor_cmd_mode", motor_cmd_mode}};
        return robot_data;
    }

    void move_to_position(std::vector<float> target_positions)
    {
        // 鑾峰彇褰撳墠鐢垫満鍙嶉浣嶇疆
        getMotorState();

        std::vector<float> current_positions(INSTALLED_MOTOR_COUNT);
        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            current_positions[i] = joint_state[i].q;
            std::cout << "Joint " << i << " current position: " << current_positions[i] << std::endl;
        }
        // wait();
        // 鎻掑€煎弬鏁?
        double total_time = 5.0;                   // 鎬绘彃鍊兼椂闂?绉?
        const double target_period = 1.0 / 1000.0; // 1ms = 0.001s (1000Hz)
        int total_steps = static_cast<int>(total_time / target_period);

        std::cout << "Starting linear interpolation to zero position..." << std::endl;
        std::cout << "Total time: " << total_time << "s, Steps: " << total_steps << ", Target frequency: 1000Hz" << std::endl;

        // 璁＄畻姣忎釜鐢垫満鐨勬彃鍊兼闀?
        std::vector<float> step_increments(INSTALLED_MOTOR_COUNT);
        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            step_increments[i] = (target_positions[i] - current_positions[i]) / total_steps;
            // std::cout << "Motor " << i << " step increment: " << step_increments[i] << std::endl;
        }

        // 鐢ㄤ簬绮剧‘鏃堕棿鎺у埗
        auto start_time = std::chrono::high_resolution_clock::now();

        // 寮€濮嬬嚎鎬ф彃鍊艰繍鍔?
        for (int step = 0; step <= total_steps; ++step)
        {
            // 璁板綍甯у紑濮嬫椂闂?
            auto frame_start = std::chrono::high_resolution_clock::now();
            // 绾挎€ф彃鍊艰绠?
            for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
            {
                // 绠€鍗曠嚎鎬ф彃鍊硷細current + step * increment
                float interpolated_pos = current_positions[i] + step * step_increments[i];
                joint_cmd[i].q = interpolated_pos;
                joint_cmd[i].dq = 0.0;            // 閫熷害鐢辨帶鍒跺櫒璁＄畻
                joint_cmd[i].tau = 0.0;           // 鍔涚煩鐢辨帶鍒跺櫒璁＄畻
                joint_cmd[i].mode = RUN_MODE_CSP; // 浣嶇疆妯″紡
            }

            // 鍙戦€佸懡浠?
            sendMotorCmd();
            // 鑾峰彇鍙嶉
            getMotorState();

            // 姣?00姝ユ墦鍗颁竴娆¤繘搴?
            // if (step % 500 == 0) {
            //     auto current_time = std::chrono::high_resolution_clock::now();
            //     auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
            //     std::cout << "Step " << step << "/" << total_steps
            //               << " (t=" << (step * target_period) << "s)"
            //               << ", Elapsed: " << elapsed.count() << "ms" << std::endl;
            //     for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i) {
            //         std::cout << "  Joint " << i << ": " << joint_cmd[i].q << std::endl;
            //     }
            // }

            // 璁＄畻鎵ц鏃堕棿骞惰皟鏁翠紤鐪?
            auto frame_end = std::chrono::high_resolution_clock::now();
            auto execution_time = std::chrono::duration<double>(frame_end - frame_start).count();

            // 璁＄畻闇€瑕佷紤鐪犵殑鏃堕棿
            double sleep_time = target_period - execution_time;
            if (sleep_time > 0)
            {
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
            }
            else
            {
                // 濡傛灉鎵ц鏃堕棿瓒呰繃鐩爣鍛ㄦ湡锛岃緭鍑鸿鍛婁絾缁х画鎵ц
                std::cerr << "Warning: Frame execution time (" << execution_time * 1000 << "ms) exceeds target period (1ms)" << std::endl;
            }
        }

        std::cout << "Linear interpolation to zero position completed!" << std::endl;

        // 楠岃瘉鏈€缁堜綅缃?
        std::cout << "Final positions:" << std::endl;
        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            std::cout << "  Joint " << i << ": " << joint_cmd[i].q << " (target: " << target_positions[i] << ")" << std::endl;
        }
    }

    void run_rl_solver()
    {
        if (sim2realCfg.save_data_flag)
        {
            data_file.open(sim2realCfg.data_path + "_sim2real_data.txt");
            std::cout << "data_file: " << sim2realCfg.data_path << std::endl;
        }

        getMotorState(); // get current motor state
        sendRLState();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        std::cout << "Move to Home Position!" << std::endl;
        std::vector<float> home_positions = {
            0.0,   // hip_joint_r_roll
            0.0,   // hip_joint_r_yaw
            -0.12, // hip_joint_r_pitch
            0.24,  // knee_joint_r
            -0.12, // ankle_joint_r_pitch
            0.0,   // ankle_joint_r_roll
            0.0,   // hip_joint_l_roll
            0.0,   // hip_joint_l_yaw
            -0.12, // hip_joint_l_pitch
            0.24,  // knee_joint_l
            -0.12, // ankle_joint_l_pitch
            0.0,   // ankle_joint_l_roll
        };

        // wait();
        move_to_position(home_positions);
        std::cout << "Move to Home Position Done!" << std::endl;
        // wait();
        sleep(2);
        std::cout << "Start RL Solver Loop!" << std::endl;

        getMotorState(); // get current motor state

        try
        {
            // ===== 鍒濆鍖栫粷瀵规椂闂村熀鍑?=====
            struct timespec next;
            clock_gettime(CLOCK_MONOTONIC, &next);
            auto last_overrun_warn_time = std::chrono::steady_clock::now();

            while (runFlag.load())
            {
                const auto loop_begin = std::chrono::steady_clock::now();
                dds_bridge_.spinOnce();

                /* ================= 鎺у埗閫昏緫寮€濮?================= */
                getRLCmd();
                if (open_rl == static_cast<int>(rl_master::kOpenRlEnabled) && !latest_cmd_fresh)
                {
                    open_rl = static_cast<int>(rl_master::kOpenRlDisabled);
                }

                if (open_rl == static_cast<int>(rl_master::kOpenRlEnabled) && last_open_rl == static_cast<int>(rl_master::kOpenRlDisabled))
                {
                    start_time = std::chrono::high_resolution_clock::now();
                }

                if (open_rl == static_cast<int>(rl_master::kOpenRlEnabled))
                {
                    getMotorState();
                    sendRLState();
                    sendMotorCmd();
                }
                else
                {
                    if (last_open_rl != open_rl)
                    {
                        std::cout << "[RL_solver] hold mode active" << std::endl;
                        getMotorState();
                    }

                    for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
                    {
                        joint_cmd[i].q = joint_state[i].q;
                        joint_cmd[i].dq = 0.0;
                        joint_cmd[i].tau = 0.0;
                        joint_cmd[i].mode = RUN_MODE_CSP;
                    }
                    sendMotorCmd();
                    sendRLState();
                }

                if (sim2realCfg.save_data_flag && open_rl == static_cast<int>(rl_master::kOpenRlEnabled))
                {
                    auto robot_state_bag = get_robot_state_bag();
                    for (const auto &[key, value] : robot_state_bag)
                    {
                        data_file << key << ": ";
                        for (size_t i = 0; i < value.size(); ++i)
                        {
                            data_file << value[i];
                            if (i < value.size() - 1)
                                data_file << ", ";
                        }
                        data_file << std::endl;
                    }
                }

                const auto loop_end = std::chrono::steady_clock::now();
                const auto loop_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_begin).count();
                if (loop_elapsed_us > sim2realCfg.loop_overrun_warn_us)
                {
                    ++loop_overrun_count_;
                    if ((loop_end - last_overrun_warn_time) > std::chrono::seconds(1))
                    {
                        std::cerr << "[RL_solver] loop overrun: " << loop_elapsed_us
                                  << " us, total overruns: " << loop_overrun_count_ << std::endl;
                        last_overrun_warn_time = loop_end;
                    }
                }

                /* ================= 鎺у埗閫昏緫缁撴潫 ================= */
                // ===== 璁＄畻涓嬩竴涓?1ms 鐨勭粷瀵瑰敜閱掓椂闂?=====
                next.tv_nsec += CONTROL_PERIOD_NS;
                if (next.tv_nsec >= 1'000'000'000)
                {
                    next.tv_nsec -= 1'000'000'000;
                    next.tv_sec++;
                }

                // ===== 闃诲鍒颁笅涓€涓帶鍒跺懆鏈燂紙鍏抽敭锛?====
                int ret = clock_nanosleep(
                    CLOCK_MONOTONIC,
                    TIMER_ABSTIME,
                    &next,
                    nullptr);

                if (ret != 0 && ret != EINTR)
                {
                    perror("clock_nanosleep");
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "program is stopped, perform motor return to zero operation..." << std::endl;
            getMotorState();
            for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
            {
                joint_cmd[i].q = joint_state[i].q;
                joint_cmd[i].dq = 0.0;
                joint_cmd[i].tau = 0.0;
                joint_cmd[i].mode = RUN_MODE_CSP;
            }
            sendMotorCmd();
        }

        getMotorState();
        for (int i = 0; i < INSTALLED_MOTOR_COUNT; ++i)
        {
            joint_cmd[i].q = joint_state[i].q;
            joint_cmd[i].dq = 0.0f;
            joint_cmd[i].tau = 0.0f;
            joint_cmd[i].mode = RUN_MODE_CSP;
        }
        sendMotorCmd();
    }

    // joint and motor data
    std::vector<JointData> joint_state; // joint state
    std::vector<JointData> joint_cmd;   // joint command
    std::vector<JointData> motor_state; // motor state
    std::vector<JointData> motor_cmd;   // motor command

    // data array
    std::vector<float> joint_cmd_q;
    std::vector<float> joint_cmd_dq;
    std::vector<float> joint_cmd_tau;
    std::vector<float> joint_state_q;
    std::vector<float> joint_state_dq;
    std::vector<float> joint_state_tau;
    std::vector<float> motor_cmd_q;
    std::vector<float> motor_cmd_dq;
    std::vector<float> motor_cmd_tau;
    std::vector<float> motor_state_q;
    std::vector<float> motor_state_dq;
    std::vector<float> motor_state_tau;
    std::vector<float> motor_cmd_mode;

    int open_rl;
    int last_open_rl;
    bool latest_cmd_fresh = true;
    uint32_t last_cmd_seq_ = 0;
    double last_stale_warn_time_s_ = 0.0;
    uint64_t loop_overrun_count_ = 0;

    MHandle motor_feedback_all[MOTOR_COUNT_MAX];
    MHandle motor_target_all[MOTOR_COUNT_MAX];

    KinConv kinConv;
    Knee_Kinematics knee_kinematics;
    Ankle_Kinematics ankle_kinematics;
    SolverDdsBridge dds_bridge_;

    Sim2realCfg sim2realCfg;
    Sim2realCfg standSim2RealCfg;

    MovingAverageFilter velocity_filters[MOTOR_COUNT_MAX]; // 涓烘瘡涓叧鑺傚垱寤烘护娉㈠櫒

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
};

void handleSignal(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\nCtrl+C detected. Closing File..." << std::endl;
        data_file.flush();
        data_file.close();
        runFlag.store(false);
    }
}

int main()
{
    set_realtime_priority(2, 90);

    auto robotSolver = RobotSolver::create();
    if (!robotSolver)
    {
        std::cerr << "Failed to create RobotSolver." << std::endl;
        return -1;
    }

    signal(SIGINT, handleSignal);

    robotSolver->robotSolver_Init();
    robotSolver->run_rl_solver();

    return 0;
}

