import os

def process_motor_state_tau_from_file(filename, start_time=0.0, interval=0.0025):
    """
    从文件读取数据并处理，提取motor_state_tau并添加时间戳
    
    参数:
    filename: 数据文件路径
    start_time: 起始时间戳（秒）
    interval: 时间间隔（秒），默认1/400s = 0.0025s
    """
    timestamp = start_time
    results = []
    
    try:
        with open(filename, 'r') as file:
            for line in file:
                line = line.strip()
                if line.startswith('motor_state_tau:'):
                    # 提取motor_state_tau后面的数值
                    values_str = line.split('motor_state_tau:')[1].strip()
                    values = [float(x.strip()) for x in values_str.split(',')]
                    
                    # 添加时间戳和数值到结果
                    results.append({
                        'timestamp': timestamp,
                        'motor_state_tau': values
                    })
                    
                    # 时间戳递增
                    timestamp += interval
        
        print(f"成功处理文件: {filename}")
        print(f"找到 {len(results)} 条 motor_state_tau 数据")
        
    except FileNotFoundError:
        print(f"错误: 文件 {filename} 不存在")
        return []
    except Exception as e:
        print(f"处理文件时出错: {e}")
        return []
    
    return results

def format_output(results, output_filename=None):
    """格式化输出结果"""
    if output_filename:
        # 输出到文件
        with open(output_filename, 'w') as f:
            # f.write("时间戳(s), motor_state_tau值\n")
            for result in results:
                timestamp = result['timestamp']
                values = result['motor_state_tau']
                values_str = ', '.join([f"{x:.6f}" for x in values])
                f.write(f"{timestamp:.6f}, {values_str}\n")
        print(f"结果已保存到: {output_filename}")
    else:
        # 输出到控制台
        print("时间戳(s), motor_state_tau值")
        for result in results:
            timestamp = result['timestamp']
            values = result['motor_state_tau']
            values_str = ', '.join([f"{x:.6f}" for x in values])
            print(f"{timestamp:.6f}, {values_str}")

def main():
    # 文件路径
    filename = "/home/nvidia/Documents/humanoid_jc/src/humanoid_rl_controller/rl_master/data/Oct28_22-08-15_jingchu01_policy_0903_sim2real_data.txt"
    
    # 检查文件是否存在
    if not os.path.exists(filename):
        print(f"文件不存在: {filename}")
        return
    
    # 处理数据
    results = process_motor_state_tau_from_file(filename)
    
    if results:
        # 输出到控制台（前5条作为示例）
        print("\n前5条数据预览:")
        format_output(results[:5])
        
        # 保存到文件
        output_file = "/home/nvidia/Documents/humanoid_jc/src/humanoid_rl_controller/rl_master/data/motor_state_tau_with_timestamp.txt"
        format_output(results, output_file)
        
        # 统计信息
        print(f"\n统计信息:")
        print(f"总数据条数: {len(results)}")
        print(f"时间跨度: {results[0]['timestamp']:.6f}s - {results[-1]['timestamp']:.6f}s")
        print(f"总时长: {(results[-1]['timestamp'] - results[0]['timestamp'] + 0.0025):.6f}s")

if __name__ == "__main__":
    main()