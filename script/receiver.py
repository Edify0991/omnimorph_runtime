import socket
import struct
import threading
import time
import sys
from typing import Callable, Optional
import requests

# 帧头和帧尾定义（4字节，使用大端序）
FRAME_HEADER = 0xA5A5A5A5
FRAME_TAIL = 0x5A5A5A5A

# 消息类型
CMD_VEL = 0x01
HEARTBEAT = 0x02

class CmdDataStruct:
    def __init__(self, 
                 linear_x: float = 0.0,
                 linear_y: float = 0.0,
                 linear_z: float = 0.0,
                 angular_x: float = 0.0,
                 angular_y: float = 0.0,
                 angular_z: float = 0.0):
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.linear_z = linear_z
        self.angular_x = angular_x
        self.angular_y = angular_y
        self.angular_z = angular_z

    def __repr__(self) -> str:
        return (f"CmdDataStruct(linear_x={self.linear_x}, linear_y={self.linear_y}, "
                f"linear_z={self.linear_z}, angular_x={self.angular_x}, "
                f"angular_y={self.angular_y}, angular_z={self.angular_z})")

class Receiver:
    def __init__(self, 
                 server_ip: str = "192.168.3.14", 
                 server_port: int = 8888,
                 client_port: int = 9999):
        self.server_ip = server_ip
        self.server_port = server_port
        self.client_port = client_port
        self.running = False
        
        # 套接字和地址
        self.sockfd = None
        self.server_addr = (server_ip, server_port)
        self.client_addr = ("0.0.0.0", client_port)  # 绑定所有接口
        
        # 回调函数
        self.cmd_callback: Optional[Callable[[CmdDataStruct], None]] = None
        self.error_callback: Optional[Callable[[str], None]] = None
        
        # 线程
        self.main_thread: Optional[threading.Thread] = None
        self.heartbeat_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

    def start(self, 
              cmd_callback: Callable[[CmdDataStruct], None], 
              error_callback: Optional[Callable[[str], None]] = None) -> bool:
        if self.running:
            return False
        
        self.cmd_callback = cmd_callback
        self.error_callback = error_callback
        
        # 创建UDP套接字
        try:
            self.sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except Exception as e:
            self._report_error(f"创建套接字失败: {e}")
            return False
        
        # 绑定客户端地址
        try:
            self.sockfd.bind(self.client_addr)
        except Exception as e:
            self._report_error(f"绑定套接字失败: {e}")
            self.sockfd.close()
            return False
        
        # 设置接收超时（1秒）
        self.sockfd.settimeout(1)
        
        # 启动线程
        self.running = True
        self.main_thread = threading.Thread(target=self._run, daemon=True)
        self.main_thread.start()
        self._start_heartbeat()
        return True

    def stop(self) -> None:
        if not self.running:
            return
        
        self.running = False
        if self.main_thread and self.main_thread.is_alive():
            self.main_thread.join()
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join()
        if self.sockfd:
            self.sockfd.close()
            self.sockfd = None

    def _run(self) -> None:
        while self.running:
            try:
                data, _ = self.sockfd.recvfrom(2048)
                # print("received!")
                if data:
                    cmd = CmdDataStruct()
                    if self._parse_packet(data, cmd):
                        with self.lock:
                            if self.cmd_callback:
                                self.cmd_callback(cmd)
            except socket.timeout:
                # 超时属于正常情况，继续循环
                # print("socket接收信息超时!")
                continue
            except Exception as e:
                self._report_error(f"接收数据失败: {e}")
        print("running", self.running)

    def _start_heartbeat(self) -> None:
        """启动心跳包发送线程"""
        def heartbeat_loop():
            while self.running:
                try:
                    # 构造心跳包（大端序）
                    # 格式: [4B帧头] [4B数据长度(1)] [1B消息类型] [4B CRC32] [4B帧尾]
                    header = struct.pack(">I", FRAME_HEADER)
                    data_len = struct.pack(">I", 1)  # 消息类型占1字节
                    msg_type = struct.pack("B", HEARTBEAT)
                    
                    # 计算CRC32（仅对消息类型字段计算）
                    crc = self._calculate_crc32(msg_type)
                    crc_bytes = struct.pack(">I", crc)
                    tail = struct.pack(">I", FRAME_TAIL)
                    
                    heartbeat_packet = header + data_len + msg_type + crc_bytes + tail
                    self.sockfd.sendto(heartbeat_packet, self.server_addr)
                except Exception as e:
                    self._report_error(f"发送心跳包失败: {e}")
                time.sleep(1)  # 每秒发送一次
        
        self.heartbeat_thread = threading.Thread(target=heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()

    def _parse_packet(self, data: bytes, cmd: CmdDataStruct) -> bool:
        """解析数据包（带帧头/帧尾校验和CRC32校验）"""
        try:
            if len(data) < 17:  # 最小包长度
                self._report_error("数据长度不足")
                return False
            
            # 1. 检查帧头
            header = struct.unpack(">I", data[:4])[0]
            if header != FRAME_HEADER:
                self._report_error("帧头错误")
                return False
            
            # 2. 检查帧尾
            tail = struct.unpack(">I", data[-4:])[0]
            if tail != FRAME_TAIL:
                self._report_error("帧尾错误")
                return False
            
            # 3. 提取数据长度
            data_len = struct.unpack(">I", data[4:8])[0]
            expected_length = 4 + 4 + 4 + 4 + data_len  # 帧头+长度+类型+CRC+数据
            if len(data) != expected_length:
                self._report_error(f"数据长度不匹配（期望{expected_length}, 实际{len(data)}）")
                return False
            
            # 4. 检查消息类型
            msg_type = data[8]
            if msg_type != CMD_VEL:
                return False  # 忽略非CMD_VEL消息
            
            # 5. 提取CRC和数据部分
            crc_received = struct.unpack(">I", data[8+data_len:12+data_len])[0]
            data_part = data[8:8+data_len]
            crc_calculated = self._calculate_crc32(data_part)
            
            if crc_received != crc_calculated:
                self._report_error("CRC32校验失败")
                return False
            
            # 6. 解析速度数据（小端序转主机字节序）
            if len(data_part) != 1 + 6 * 4:  # 1Byte消息类型，6个float，每个4字节，
                self._report_error("速度数据长度错误")
                return False
            
            # 解包6个float（小端序）
            cmd_data_part = data_part[1:]  # 跳过消息类型字节
            cmd_data = struct.unpack("<6f", cmd_data_part)
            cmd.linear_x, cmd.linear_y, cmd.linear_z, \
            cmd.angular_x, cmd.angular_y, cmd.angular_z = cmd_data
            return True
            
        except Exception as e:
            self._report_error(f"解析数据包失败: {e}")
            return False

    def _calculate_crc32(self, data: bytes) -> int:
        """计算CRC32校验值（简化实现，与C++版本兼容）"""
        crc = 0xFFFFFFFF
        for byte in data:
            crc = (crc >> 8) ^ self._crc_table[(crc ^ byte) & 0xFF]
        return crc ^ 0xFFFFFFFF

    # CRC32表（与C++代码中的表一致，仅列出前4项，完整表需补全）
    _crc_table = [0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
                    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
                    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
                    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
                    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
                    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
                    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
                    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
                    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
                    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
                    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
                    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
                    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
                    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
                    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
                    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
                    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
                    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
                    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
                    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
                    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
                    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
                    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
                    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
                    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
                    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
                    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
                    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
                    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
                    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
                    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
                    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
    ]

    def _report_error(self, msg: str) -> None:
        """报告错误到回调函数"""
        with self.lock:
            if self.error_callback:
                self.error_callback(msg)
    
    def test_get_navi_status(self, server_url):
        url = f"{server_url}/getNaviStatus"
        try:
            response = requests.get(url)
            if response.status_code == 200:
                data = response.json()
                if data.get("status") == "success":
                    status = data.get("navigation_status")
                    status_map = {
                    0: "等待目标点",
                    1: "正在运行",
                    3: "导航成功",
                    4: "导航失败"
                    }
                    # print(f"导航状态获取成功: {status_map.get(status, '未知状态')}")
                else:
                    print("导航状态获取失败:", data.get("message"))
            else:
                print(f"请求失败， 状态码: {response.status_code}")
        except Exception as e:
            print(f"请求失败: {e}")
        return status


# 示例用法
if __name__ == "__main__":
    server_port = 8888
    client_port = 9999
    server_ip = "0.0.0.0"
    
    if len(sys.argv) == 4:
        print("用法: python client.py <server_ip> <server_port> <client_port>")
        server_ip = sys.argv[1]
        server_port = int(sys.argv[2])
        client_port = int(sys.argv[3])
        
    elif len(sys.argv) == 3:
        print("用法: python client.py <server_ip> <server_port> <client_port=9999>")
        server_ip = sys.argv[1]
        server_port = int(sys.argv[2])
        client_port = 9999
    elif len(sys.argv) == 2:
        if sys.argv[1] == "--help" or sys.argv[1] == "-h":
            print("用法: python client.py <server_ip> <server_port=8888> <client_port=9999>")
            print("默认服务器IP: 192.168.2.251, 服务器端口: 8888, 客户端端口: 9999")
            sys.exit(0)
            
        print("用法: python client.py <server_ip> <server_port=8888> <client_port=9999>")
        server_ip = sys.argv[1]
        server_port = 8888
        client_port = 9999
    elif len(sys.argv) == 1:
        print("用法: python client.py <server_ip=192.168.2.251> <server_port=8888> <client_port=9999>")
        server_ip = "192.168.2.251"
        server_port = 8888
        client_port = 9999

    def cmd_callback(cmd: CmdDataStruct) -> None:
        print(f"收到速度指令: {cmd}")

    def error_callback(msg: str) -> None:
        print(f"错误: {msg}")


    receiver = Receiver(server_ip="192.168.2.251", server_port=8888, client_port=9999)
    receiver.start(cmd_callback, error_callback)
    
    try:
        while True:
            time.sleep(100)
    except KeyboardInterrupt:
        receiver.stop()
        print("已停止接收")
